"""Strict, machine-readable task registry validation."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, Mapping, Optional

import yaml


class RegistryError(ValueError):
    """Raised for a registry which cannot safely be advertised as runnable."""


class _UniqueKeyLoader(yaml.SafeLoader):
    pass


def _construct_mapping(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise RegistryError(f"duplicate YAML key: {key!r}")
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeyLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_mapping)


def load_yaml(path: Path) -> Mapping[str, object]:
    try:
        with path.open("r", encoding="utf-8") as stream:
            data = yaml.load(stream, Loader=_UniqueKeyLoader)
    except OSError as exc:
        raise RegistryError(f"could not read {path}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise RegistryError(f"invalid YAML in {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise RegistryError(f"{path} must contain a mapping")
    return data


@dataclass(frozen=True)
class TaskDefinition:
    task_id: str
    display_name: str
    availability: str
    executor: str = ""
    route_package: str = ""
    route: str = ""
    route_key: str = ""
    frame_id: str = ""
    nav2_profile: str = ""
    control_policy: str = ""
    reason: str = ""
    supports_dry_run: bool = True
    supports_auto_mode: bool = True
    features: Mapping[str, object] = field(default_factory=dict)

    @property
    def runnable(self) -> bool:
        return self.availability in {"available", "experimental"}


# These are live safety inputs.  Route validation cannot prove them.
RUNTIME_READINESS_FEATURES = frozenset({"buoy_perception"})


def required_runtime_readiness(task: TaskDefinition) -> frozenset[str]:
    return frozenset(
        name for name in RUNTIME_READINESS_FEATURES if task.features.get(name) is True
    )


class TaskRegistry:
    """Validated task definitions plus the reasons non-runnable tasks remain visible."""

    KNOWN_AVAILABILITY = {"available", "experimental", "not_implemented", "disabled"}

    def __init__(self, definitions: Iterable[TaskDefinition]) -> None:
        definitions = tuple(definitions)
        self._tasks = {item.task_id: item for item in definitions}
        if len(self._tasks) != len(definitions):
            raise RegistryError("task IDs must be unique")

    def get(self, task_id: str) -> Optional[TaskDefinition]:
        return self._tasks.get(task_id)

    def all(self) -> tuple[TaskDefinition, ...]:
        return tuple(self._tasks.values())

    @classmethod
    def from_file(cls, path: Path, *, package_shares: Optional[Mapping[str, Path]] = None) -> "TaskRegistry":
        data = load_yaml(path)
        allowed_top = {"nav2_profiles", "control_policies", "tasks"}
        unknown = set(data).difference(allowed_top)
        if unknown:
            raise RegistryError(f"unknown registry keys: {sorted(unknown)}")
        task_entries = data.get("tasks")
        if not isinstance(task_entries, dict) or not task_entries:
            raise RegistryError("registry must contain a non-empty tasks mapping")
        nav2_profiles = _string_set(data.get("nav2_profiles", []), "nav2_profiles")
        policies = _string_set(data.get("control_policies", []), "control_policies")
        resolver = _PackageShareResolver(package_shares or {}, path.parent)
        tasks = []
        for task_id, raw in task_entries.items():
            tasks.append(_parse_task(str(task_id), raw, resolver, nav2_profiles, policies))
        return cls(tasks)


def _string_set(value: object, name: str) -> set[str]:
    if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
        raise RegistryError(f"{name} must be a list of strings")
    return set(value)


class _PackageShareResolver:
    def __init__(self, package_shares: Mapping[str, Path], base_path: Path) -> None:
        self._shares = {name: Path(value) for name, value in package_shares.items()}
        self._base_path = base_path

    def route_path(self, package: str, route: str) -> Path:
        if Path(route).is_absolute() or ".." in Path(route).parts:
            raise RegistryError("route must be a safe relative path")
        if package:
            root = self._shares.get(package)
            if root is None:
                try:
                    from ament_index_python.packages import get_package_share_directory

                    root = Path(get_package_share_directory(package))
                except Exception as exc:
                    raise RegistryError(f"could not resolve route package {package!r}: {exc}") from exc
        else:
            root = self._base_path
        candidate = root / route
        if not candidate.is_file():
            raise RegistryError(f"route does not exist: {candidate}")
        return candidate


def _parse_task(
    task_id: str,
    raw: object,
    resolver: _PackageShareResolver,
    nav2_profiles: set[str],
    policies: set[str],
) -> TaskDefinition:
    if not isinstance(raw, dict):
        raise RegistryError(f"task {task_id!r} must be a mapping")
    allowed = {
        "display_name", "availability", "executor", "route_package", "route", "route_key",
        "frame_id", "nav2_profile", "control_policy", "reason", "supports_dry_run",
        "supports_auto_mode", "features",
    }
    unknown = set(raw).difference(allowed)
    if unknown:
        raise RegistryError(f"task {task_id!r} has unknown keys: {sorted(unknown)}")
    display_name = _required_string(raw, "display_name", task_id)
    availability = _required_string(raw, "availability", task_id)
    if availability not in TaskRegistry.KNOWN_AVAILABILITY:
        raise RegistryError(f"task {task_id!r} has invalid availability {availability!r}")
    reason = _optional_string(raw, "reason")
    if not task_id or task_id.strip() != task_id:
        raise RegistryError("task ID must be a non-empty trimmed string")
    if not availability in {"available", "experimental"}:
        if not reason:
            raise RegistryError(f"non-runnable task {task_id!r} requires a reason")
        return TaskDefinition(task_id, display_name, availability, reason=reason)
    executor = _required_string(raw, "executor", task_id)
    if executor not in {"dummy", "waypoint_sequence", "staged_docking", "task2_mppi"}:
        raise RegistryError(f"task {task_id!r} references unknown executor {executor!r}")
    route_package = _optional_string(raw, "route_package")
    route = _required_string(raw, "route", task_id)
    route_key = _required_string(raw, "route_key", task_id)
    resolver.route_path(route_package, route)
    nav2_profile = _required_string(raw, "nav2_profile", task_id)
    control_policy = _required_string(raw, "control_policy", task_id)
    if nav2_profile not in nav2_profiles:
        raise RegistryError(f"task {task_id!r} references unknown Nav2 profile {nav2_profile!r}")
    if control_policy not in policies:
        raise RegistryError(f"task {task_id!r} references unknown policy {control_policy!r}")
    features = raw.get("features", {})
    if not isinstance(features, dict) or not all(isinstance(key, str) for key in features):
        raise RegistryError(f"task {task_id!r} features must be a mapping")
    return TaskDefinition(
        task_id=task_id,
        display_name=display_name,
        availability=availability,
        executor=executor,
        route_package=route_package,
        route=route,
        route_key=route_key,
        frame_id=_required_string(raw, "frame_id", task_id),
        nav2_profile=nav2_profile,
        control_policy=control_policy,
        reason=reason,
        supports_dry_run=_optional_bool(raw, "supports_dry_run", True),
        supports_auto_mode=_optional_bool(raw, "supports_auto_mode", True),
        features=features,
    )


def _required_string(raw: Mapping[str, object], key: str, task_id: str) -> str:
    value = raw.get(key)
    if not isinstance(value, str) or not value.strip():
        raise RegistryError(f"task {task_id!r} requires non-empty string {key!r}")
    return value


def _optional_string(raw: Mapping[str, object], key: str) -> str:
    value = raw.get(key, "")
    if not isinstance(value, str):
        raise RegistryError(f"{key!r} must be a string")
    return value


def _optional_bool(raw: Mapping[str, object], key: str, default: bool) -> bool:
    value = raw.get(key, default)
    if not isinstance(value, bool):
        raise RegistryError(f"{key!r} must be a bool")
    return value
