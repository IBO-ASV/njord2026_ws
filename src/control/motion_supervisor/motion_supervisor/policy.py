"""Strict control-policy parsing and ROS-independent safety decisions."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

import yaml


class PolicyError(ValueError):
    """Raised when a policy cannot safely be used."""


class _UniqueKeyLoader(yaml.SafeLoader):
    pass


def _construct_mapping(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise PolicyError(f"duplicate YAML key: {key!r}")
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeyLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_mapping)


COMMON_BOOLEAN_REQUIREMENTS = frozenset(
    {
        "require_emergency_stop_clear",
        "require_nav2_ready",
        "require_task_ready",
        "require_ground_station",
        "require_driver_heartbeat",
        "require_localization_heartbeat",
        "require_rtk_fix",
        "require_no_critical_health_fault",
    }
)
TASK_BOOLEAN_REQUIREMENTS = frozenset(
    {
        "require_waypoint_plan",
        "require_collision_monitor",
        "require_buoy_perception",
        "require_dynamic_gate_tf",
    }
)
TASK_POLICY_REQUIREMENTS = TASK_BOOLEAN_REQUIREMENTS | {"require_ground_station"}
HEALTH_REQUIREMENT_SIGNALS = {
    "require_ground_station": "ground_station_heartbeat",
    "require_driver_heartbeat": "driver_heartbeat",
    "require_localization_heartbeat": "local_odometry",
    "require_rtk_fix": "gnss",
}


@dataclass(frozen=True)
class ControlPolicy:
    common: Mapping[str, bool]
    tasks: Mapping[str, Mapping[str, bool]]
    nav_command_timeout_sec: float
    health_signals: frozenset[str]

    def requirements_for(self, task_policy: str) -> Mapping[str, bool]:
        if not task_policy:
            return self.common
        if task_policy not in self.tasks:
            raise PolicyError(f"unknown task control policy: {task_policy!r}")
        return {**self.common, **self.tasks[task_policy]}


def _mapping(value: object, label: str) -> Mapping[str, object]:
    if not isinstance(value, dict):
        raise PolicyError(f"{label} must be a mapping")
    return value


def _bools(raw: Mapping[str, object], allowed: frozenset[str], label: str) -> dict[str, bool]:
    unknown = set(raw).difference(allowed)
    if unknown:
        raise PolicyError(f"{label} has unknown keys: {sorted(unknown)!r}")
    missing = allowed.difference(raw)
    if missing:
        raise PolicyError(f"{label} is missing required keys: {sorted(missing)!r}")
    result: dict[str, bool] = {}
    for key in allowed:
        value = raw[key]
        if type(value) is not bool:
            raise PolicyError(f"{label}.{key} must be a boolean")
        result[key] = value
    return result


def load_policy(path: Path) -> ControlPolicy:
    """Load one complete policy file, rejecting ambiguous or partial input."""
    try:
        with path.open("r", encoding="utf-8") as stream:
            raw = yaml.load(stream, Loader=_UniqueKeyLoader)
    except OSError as exc:
        raise PolicyError(f"could not read policy {path}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise PolicyError(f"invalid policy YAML {path}: {exc}") from exc
    root = _mapping(raw, str(path))
    if set(root) != {"control_policy"}:
        raise PolicyError("policy root must contain only 'control_policy'")
    policy = _mapping(root["control_policy"], "control_policy")
    if set(policy) != {"common", "task1", "task2", "task3", "return_home", "health_signals"}:
        raise PolicyError(
            "control_policy must contain exactly common, task1, task2, task3, return_home, and health_signals"
        )

    common_raw = _mapping(policy["common"], "control_policy.common")
    expected_common = COMMON_BOOLEAN_REQUIREMENTS | {"nav_command_timeout_sec"}
    if set(common_raw) != expected_common:
        unknown = set(common_raw).difference(expected_common)
        missing = expected_common.difference(common_raw)
        raise PolicyError(
            f"control_policy.common keys invalid; unknown={sorted(unknown)!r}, missing={sorted(missing)!r}"
        )
    common = _bools(
        {key: common_raw[key] for key in COMMON_BOOLEAN_REQUIREMENTS},
        COMMON_BOOLEAN_REQUIREMENTS,
        "control_policy.common",
    )
    timeout = common_raw["nav_command_timeout_sec"]
    if type(timeout) not in (int, float) or isinstance(timeout, bool) or timeout <= 0:
        raise PolicyError("control_policy.common.nav_command_timeout_sec must be a positive number")

    signals = policy["health_signals"]
    if not isinstance(signals, list) or not signals or any(not isinstance(item, str) or not item for item in signals):
        raise PolicyError("control_policy.health_signals must be a non-empty list of names")
    if len(signals) != len(set(signals)):
        raise PolicyError("control_policy.health_signals contains duplicate names")
    signal_set = frozenset(signals)
    for requirement, signal in HEALTH_REQUIREMENT_SIGNALS.items():
        if common[requirement] and signal not in signal_set:
            raise PolicyError(
                f"{requirement} requires unknown health signal {signal!r}; add it to health_signals"
            )

    tasks = {
        name: _bools(_mapping(policy[name], f"control_policy.{name}"), TASK_POLICY_REQUIREMENTS,
                     f"control_policy.{name}")
        for name in ("task1", "task2", "task3", "return_home")
    }
    return ControlPolicy(common, tasks, float(timeout), signal_set)


@dataclass(frozen=True)
class SafetyInputs:
    emergency_stop: bool = True
    nav2_ready: bool = False
    task_ready: bool = False
    health_states: Mapping[str, int] = None  # HealthSignal constants are supplied by the node.
    health_summary_critical: bool = False
    task_requirements_ready: bool = False


@dataclass(frozen=True)
class SafetyDecision:
    auto_permitted: bool
    reasons: tuple[tuple[int, str], ...]


def evaluate_auto_permission(
    requirements: Mapping[str, bool], inputs: SafetyInputs, *, health_ok_state: int, health_disabled_state: int,
    inhibit_codes: Mapping[str, int],
) -> SafetyDecision:
    """Evaluate enabled requirements only; every failed predicate has a reason."""
    reasons: list[tuple[int, str]] = []
    if requirements["require_emergency_stop_clear"] and inputs.emergency_stop:
        reasons.append((inhibit_codes["emergency_stop"], "emergency stop is active"))
    if requirements["require_nav2_ready"] and not inputs.nav2_ready:
        reasons.append((inhibit_codes["nav2_not_ready"], "Nav2 is not ready"))
    if requirements["require_task_ready"] and not inputs.task_ready:
        reasons.append((inhibit_codes["task_not_ready"], "task is not ready"))
    states = inputs.health_states or {}
    for requirement, signal in HEALTH_REQUIREMENT_SIGNALS.items():
        # A DISABLED monitor is observable but can never satisfy an enabled control requirement.
        if requirements[requirement] and states.get(signal) != health_ok_state:
            reasons.append((inhibit_codes[requirement], f"required health signal {signal!r} is not OK"))
    if requirements["require_no_critical_health_fault"] and inputs.health_summary_critical:
        reasons.append((inhibit_codes["critical_health"], "critical health fault is active"))
    if any(requirements.get(key, False) for key in TASK_BOOLEAN_REQUIREMENTS) and not inputs.task_requirements_ready:
        reasons.append((inhibit_codes["task_requirement"], "task-specific requirement is not ready"))
    return SafetyDecision(not reasons, tuple(reasons))
