"""Typed validation for the legacy waypoint YAML files.

The migration deliberately reads the existing waypoint_publisher configuration
files rather than changing their coordinates.  Once equivalence tests are in
place these files can move into this package without changing this API.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from itertools import combinations
from math import hypot, isfinite
from pathlib import Path
import re
from typing import Mapping

from .task_registry import RegistryError, load_yaml
from .geodesy import HomeDatum, wgs84_to_enu


@dataclass(frozen=True)
class Waypoint:
    waypoint_id: str
    x: float
    y: float
    yaw: float
    name: str
    waypoint_type: str
    gate_pair: object = None
    # Competition checkpoint identity is distinct from the sequential YAML
    # id (for example, sequential id 13 is competition GPS3).  Mission
    # Manager needs this metadata to apply Task1 stage safety behavior.
    competition_id: str = ""
    latitude: float | None = None
    longitude: float | None = None
    altitude: float = 0.0
    docking_motion: str = ""


@dataclass(frozen=True)
class GeodeticPoint:
    latitude: float
    longitude: float
    altitude: float = 0.0


@dataclass(frozen=True)
class Route:
    frame_id: str
    waypoints: tuple[Waypoint, ...]
    stages: Mapping[str, tuple[str, ...]]
    full_sequence_stages: Mapping[str, tuple[str, ...]]
    constraints: Mapping[str, object]

    def stage(self, name: str, *, full_sequence: bool = False) -> tuple[Waypoint, ...]:
        stage_set = self.full_sequence_stages if full_sequence else self.stages
        ids = stage_set.get(name, ())
        by_id = {waypoint.waypoint_id: waypoint for waypoint in self.waypoints}
        return tuple(by_id[item] for item in ids)

    def projection_points(self) -> tuple[GeodeticPoint, ...]:
        """GPS points which must be converted by navsat_transform /fromLL."""
        return tuple(
            GeodeticPoint(w.latitude, w.longitude, w.altitude)
            for w in self.waypoints
        )

    def with_projected_points(self, points: tuple[tuple[float, float], ...]) -> "Route":
        """Return map-coordinate waypoints after /fromLL conversion."""
        if len(points) != len(self.waypoints):
            raise RegistryError("latitude/longitude requires one projected point per waypoint")
        waypoints = tuple(
            replace(w, x=position[0], y=position[1])
            for w, position in zip(self.waypoints, points)
        )
        return replace(self, waypoints=waypoints)

    def projection_is_degenerate(self, points: tuple[tuple[float, float], ...]) -> bool:
        """Return true when a geographic route collapsed during ``/fromLL``.

        ``navsat_transform`` can briefly answer ``(0, 0)`` for every request
        before it has initialized its datum.  A zero coordinate is valid by
        itself, so reject only a collapsed *multi-point* route whose original
        geographic points span a meaningful distance.
        """
        if len(points) != len(self.waypoints) or not all(
            isfinite(x) and isfinite(y) for x, y in points
        ):
            return True
        if len(points) < 2:
            return False
        # 1e-5 degrees is about one metre at these latitudes.  Do not reject
        # deliberately repeated waypoints, but catch a partial collapse too.
        return any(
            hypot(a.latitude - b.latitude, a.longitude - b.longitude) > 1.0e-5
            and hypot(pa[0] - pb[0], pa[1] - pb[1]) < 1.0
            for (a, pa), (b, pb) in combinations(zip(self.waypoints, points), 2)
        )


class WaypointConfigLoader:
    """Loads one named route config, without sending any Nav2 goal."""

    def __init__(self, home_datum: HomeDatum | None = None) -> None:
        self._home_datum = home_datum

    def load(self, path: Path, route_key: str) -> Route:
        root = load_yaml(path)
        raw = root.get(route_key)
        if not isinstance(raw, dict):
            raise RegistryError(f"route key {route_key!r} is absent from {path}")
        allowed = {
            "frame_id", "publish_rate_hz", "waypoints", "constraints",
            "scenario", "stages", "full_sequence_stages",
        }
        unknown = set(raw).difference(allowed)
        if unknown:
            raise RegistryError(f"route {route_key!r} has unknown keys: {sorted(unknown)}")
        frame_id = raw.get("frame_id")
        if not isinstance(frame_id, str) or not frame_id:
            raise RegistryError(f"route {route_key!r} requires a non-empty frame_id")
        waypoints = self._waypoints(raw.get("waypoints"), route_key)
        ids = {waypoint.waypoint_id for waypoint in waypoints}
        stages = self._stages(raw.get("stages", {}), ids, route_key, "stages")
        full_stages = self._stages(
            raw.get("full_sequence_stages", {}), ids, route_key, "full_sequence_stages"
        )
        constraints = raw.get("constraints", {})
        if not isinstance(constraints, dict):
            raise RegistryError(f"route {route_key!r} constraints must be a mapping")
        return Route(
            frame_id, tuple(waypoints), stages, full_stages, constraints,
        )

    @staticmethod
    def _coordinate(value: object, low: float, high: float, hemispheres: str) -> float:
        if isinstance(value, bool):
            raise RegistryError("coordinate must be numeric or DMS text")
        if isinstance(value, (int, float)):
            coordinate = float(value)
        elif isinstance(value, str):
            match = re.fullmatch(
                r"\s*(\d+)°(\d+)[′'](\d+(?:\.\d+)?)[″\"]\s*([NSEW])\s*", value
            )
            if not match or match.group(4) not in hemispheres:
                raise RegistryError(f"invalid DMS coordinate {value!r}")
            degrees, minutes, seconds = (
                int(match.group(1)), int(match.group(2)), float(match.group(3))
            )
            if minutes >= 60 or not 0.0 <= seconds < 60.0:
                raise RegistryError(f"invalid DMS coordinate {value!r}")
            coordinate = degrees + minutes / 60.0 + seconds / 3600.0
            if match.group(4) in "SW":
                coordinate = -coordinate
        else:
            raise RegistryError("coordinate must be numeric or DMS text")
        if not isfinite(coordinate) or not low <= coordinate <= high:
            raise RegistryError("coordinate out of range")
        return coordinate

    @staticmethod
    def _geodetic_point(raw: object, route_key: str, label: str, *, required: bool) -> GeodeticPoint | None:
        if raw is None and not required:
            return None
        if not isinstance(raw, dict):
            raise RegistryError(f"route {route_key!r} {label} must be a mapping with latitude/longitude")
        has_latitude = "latitude" in raw
        has_longitude = "longitude" in raw
        if not required and not has_latitude and not has_longitude:
            return None
        if has_latitude != has_longitude:
            raise RegistryError(f"route {route_key!r} {label} requires both latitude and longitude")
        values = {}
        for name, low, high, hemispheres in (
            ("latitude", -90.0, 90.0, "NS"), ("longitude", -180.0, 180.0, "EW")
        ):
            try:
                values[name] = WaypointConfigLoader._coordinate(
                    raw.get(name), low, high, hemispheres
                )
            except RegistryError as exc:
                raise RegistryError(
                    f"route {route_key!r} {label} has invalid {name}: {exc}"
                ) from exc
        altitude = raw.get("altitude", 0.0)
        if isinstance(altitude, bool) or not isinstance(altitude, (int, float)) or not isfinite(altitude):
            raise RegistryError(f"route {route_key!r} {label} has invalid altitude")
        return GeodeticPoint(values["latitude"], values["longitude"], float(altitude))

    def _waypoints(self, raw: object, route_key: str) -> list[Waypoint]:
        if not isinstance(raw, list) or not raw:
            raise RegistryError(f"route {route_key!r} requires non-empty waypoints")
        result = []
        seen = set()
        for index, item in enumerate(raw):
            if not isinstance(item, dict):
                raise RegistryError(f"route {route_key!r} waypoint {index} must be a mapping")
            waypoint_id = item.get("id")
            if isinstance(waypoint_id, bool) or waypoint_id is None:
                raise RegistryError(f"route {route_key!r} waypoint {index} has invalid id")
            waypoint_id = str(waypoint_id)
            if not waypoint_id or waypoint_id in seen:
                raise RegistryError(f"route {route_key!r} waypoint IDs must be unique")
            seen.add(waypoint_id)
            values = {}
            for name in ("yaw",):
                value = item.get(name)
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not isfinite(value):
                    raise RegistryError(
                        f"route {route_key!r} waypoint {waypoint_id!r} has non-finite {name}"
                    )
                values[name] = float(value)
            name = item.get("name", waypoint_id)
            waypoint_type = item.get("type", "waypoint")
            if not isinstance(name, str) or not isinstance(waypoint_type, str):
                raise RegistryError(f"route {route_key!r} waypoint {waypoint_id!r} has invalid metadata")
            competition_id = item.get("competition_id", waypoint_id)
            if isinstance(competition_id, bool) or not isinstance(competition_id, (str, int, float)):
                raise RegistryError(
                    f"route {route_key!r} waypoint {waypoint_id!r} has invalid competition_id"
                )
            docking_motion = item.get("docking_motion", "")
            if docking_motion not in {"", "forward", "sway_negative", "omni"}:
                raise RegistryError(
                    f"route {route_key!r} waypoint {waypoint_id!r} has invalid docking_motion"
                )
            geodetic = self._geodetic_point(
                item, route_key, f"waypoint {waypoint_id!r}", required=True
            )
            x, y = 0.0, 0.0
            if self._home_datum is not None:
                x, y = wgs84_to_enu(geodetic.latitude, geodetic.longitude, self._home_datum)
            result.append(Waypoint(
                waypoint_id, x, y, values["yaw"],
                name, waypoint_type, item.get("gate_pair"), str(competition_id),
                latitude=None if geodetic is None else geodetic.latitude,
                longitude=None if geodetic is None else geodetic.longitude,
                altitude=0.0 if geodetic is None else geodetic.altitude,
                docking_motion=docking_motion,
            ))
        return result

    @staticmethod
    def _stages(
        raw: object, ids: set[str], route_key: str, field: str
    ) -> Mapping[str, tuple[str, ...]]:
        if not isinstance(raw, dict):
            raise RegistryError(f"route {route_key!r} {field} must be a mapping")
        result = {}
        for stage_name, waypoint_ids in raw.items():
            if not isinstance(stage_name, str) or not isinstance(waypoint_ids, list):
                raise RegistryError(f"route {route_key!r} {field} contains an invalid stage")
            converted = tuple(str(item) for item in waypoint_ids)
            missing = set(converted).difference(ids)
            if missing:
                raise RegistryError(
                    f"route {route_key!r} stage {stage_name!r} references missing IDs {sorted(missing)}"
                )
            result[stage_name] = converted
        return result
