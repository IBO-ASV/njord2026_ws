"""Validated, revision-checked map edits; no client-supplied filesystem paths."""

import hashlib
import math
import os
import tempfile
from pathlib import Path

import yaml

BUOY_TYPES = ("RED", "GREEN", "N", "E", "S", "W")


def revision(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate_edit(original, edited):
    if not isinstance(edited, dict) or not isinstance(edited.get("waypoints"), list):
        raise ValueError("waypoints must be a list")
    if any(not isinstance(w, dict) for w in edited["waypoints"]):
        raise ValueError("waypoints must contain mappings")
    if [w.get("id") for w in edited["waypoints"]] != [
        w["id"] for w in original["waypoints"]
    ]:
        raise ValueError("waypoint IDs/order cannot be changed")
    buoys = edited.get("buoys")
    if not isinstance(buoys, list) or len(buoys) > 200:
        raise ValueError("buoys must be a list of at most 200 entries")
    if any(not isinstance(b, dict) for b in buoys):
        raise ValueError("buoys must contain mappings")
    for point in [*edited["waypoints"], *buoys]:
        for key, bound in (("latitude", 90), ("longitude", 180)):
            value = point.get(key)
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or abs(value) > bound
            ):
                raise ValueError(f"invalid {key}")
    if any(b.get("type") not in BUOY_TYPES for b in buoys):
        raise ValueError("unknown buoy type")
    if len({b.get("id") for b in buoys}) != len(buoys) or any(
        not isinstance(b.get("id"), str) for b in buoys
    ):
        raise ValueError("buoy IDs must be unique strings")


def atomic_save(path, text, expected_revision):
    path = Path(path).resolve()
    if revision(path) != expected_revision:
        raise ValueError("YAML changed externally; reload before saving")
    mode = path.stat().st_mode & 0o777
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w") as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary, mode)
        # Check again after validation/serialization, before replacing the file.
        if revision(path) != expected_revision:
            raise ValueError("YAML changed externally; reload before saving")
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def waypoint_yaml_edit(text, route_key, waypoints, buoys):
    """Patch scalar spans, preserving comments, aliases and all mission metadata."""
    root = yaml.compose(text)

    def mapping(node):
        return {key.value: value for key, value in node.value}

    entries = mapping(root)
    nodes = mapping(entries[route_key])["waypoints"].value
    if len(nodes) != len(waypoints):
        raise ValueError("waypoint count changed")
    replacements = []
    for node, point in zip(nodes, waypoints):
        fields = mapping(node)
        for key in ("latitude", "longitude"):
            scalar = fields[key]
            replacements.append(
                (
                    scalar.start_mark.index,
                    scalar.end_mark.index,
                    repr(float(point[key])),
                )
            )
    raw = yaml.safe_load(text)
    all_buoys = raw.get("map_buoys", {})
    all_buoys[route_key] = buoys
    value = yaml.safe_dump(
        all_buoys, default_flow_style=True, allow_unicode=True, width=100000
    ).strip()
    if "map_buoys" in entries:
        node = entries["map_buoys"]
        replacements.append((node.start_mark.index, node.end_mark.index, value + "\n"))
    else:
        text += "\nmap_buoys: " + value + "\n"
    # Aliased routes may reference the same node; do not patch a scalar twice.
    for start, end, replacement in sorted(set(replacements), reverse=True):
        text = text[:start] + replacement + text[end:]
    yaml.safe_load(text)
    return text
