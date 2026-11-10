#!/usr/bin/env python3
"""Load and validate role-owned heartbeat/health inventories.

The inventory deliberately contains observation details only.  AUTO inhibition
requirements are resolved by the control policy using the signal names here.
Keeping this parser small and dependency-free makes it usable from launch code,
unit tests, and offline configuration checks.
"""

from __future__ import annotations

from dataclasses import dataclass
import pathlib
from typing import Any, Mapping

import yaml


class InventoryError(ValueError):
    """Raised when a role heartbeat inventory is malformed."""


@dataclass(frozen=True)
class SignalConfig:
    name: str
    topic: str
    message_type: str
    timeout_sec: float
    expected_frequency_hz: float
    minimum_frequency_hz: float
    enabled: bool = True


@dataclass(frozen=True)
class RoleInventory:
    role: str
    signals: tuple[SignalConfig, ...]

    @property
    def by_name(self) -> dict[str, SignalConfig]:
        return {signal.name: signal for signal in self.signals}


_REQUIRED_SIGNAL_KEYS = frozenset(
    {"topic", "type", "timeout_sec", "expected_frequency_hz"}
)


def _fail(path: pathlib.Path, message: str) -> InventoryError:
    return InventoryError(f"{path}: {message}")


def parse_inventory(data: Any, *, source: str | pathlib.Path = "<inventory>") -> RoleInventory:
    """Validate an inventory mapping and return a typed immutable model."""

    path = pathlib.Path(source)
    if not isinstance(data, Mapping):
        raise _fail(path, "top-level value must be a mapping")
    role = data.get("role")
    signals = data.get("signals")
    if not isinstance(role, str) or not role.strip():
        raise _fail(path, "'role' must be a non-empty string")
    if not isinstance(signals, Mapping) or not signals:
        raise _fail(path, "'signals' must be a non-empty mapping")

    parsed: list[SignalConfig] = []
    for name, raw in signals.items():
        if not isinstance(name, str) or not name.strip():
            raise _fail(path, "signal names must be non-empty strings")
        if not isinstance(raw, Mapping):
            raise _fail(path, f"signal '{name}' must be a mapping")
        missing = _REQUIRED_SIGNAL_KEYS - raw.keys()
        if missing:
            raise _fail(path, f"signal '{name}' missing keys: {sorted(missing)}")
        topic = raw["topic"]
        message_type = raw["type"]
        if not isinstance(topic, str) or not topic.startswith("/"):
            raise _fail(path, f"signal '{name}' topic must be an absolute ROS topic")
        if not isinstance(message_type, str) or "/" not in message_type:
            raise _fail(path, f"signal '{name}' type must look like package/msg/Type")
        try:
            timeout = float(raw["timeout_sec"])
            expected = float(raw["expected_frequency_hz"])
            minimum = float(raw.get("minimum_frequency_hz", expected))
        except (TypeError, ValueError) as error:
            raise _fail(path, f"signal '{name}' numeric values are invalid") from error
        if timeout <= 0:
            raise _fail(path, f"signal '{name}' timeout_sec must be positive")
        if minimum < 0 or minimum > expected:
            raise _fail(path, f"signal '{name}' minimum_frequency_hz must be between 0 and expected_frequency_hz")
        enabled = raw.get("enabled", True)
        if not isinstance(enabled, bool):
            raise _fail(path, f"signal '{name}' enabled must be boolean")
        parsed.append(
            SignalConfig(name, topic, message_type, timeout, expected, minimum, enabled)
        )
    return RoleInventory(role.strip(), tuple(parsed))


def load_inventory(path: str | pathlib.Path) -> RoleInventory:
    """Read YAML from *path* and validate it strictly."""

    source = pathlib.Path(path)
    try:
        with source.open(encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
    except OSError as error:
        raise InventoryError(f"{source}: cannot read inventory: {error}") from error
    except yaml.YAMLError as error:
        raise InventoryError(f"{source}: invalid YAML: {error}") from error
    return parse_inventory(data, source=source)


def inventory_parameters(inventory: RoleInventory) -> list[dict[str, Any]]:
    """Convert enabled signals into TopicHeartbeatMonitor parameters."""

    return [
        {
            "monitor_name": signal.name,
            "topic": signal.topic,
            "topic_type": signal.message_type,
            "mode": "required_frequency",
            "expected_frequency": signal.expected_frequency_hz,
            "minimum_frequency": signal.minimum_frequency_hz,
            "timeout": signal.timeout_sec,
            "stale_timeout": signal.timeout_sec * 3.0,
        }
        for signal in inventory.signals
        if signal.enabled
    ]


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inventory", type=pathlib.Path)
    args = parser.parse_args()
    loaded = load_inventory(args.inventory)
    print(f"{loaded.role}: {len(loaded.signals)} signals ({len(inventory_parameters(loaded))} enabled)")
