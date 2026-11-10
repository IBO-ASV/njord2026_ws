"""Validation tests for role-owned heartbeat inventories."""

from pathlib import Path
import importlib.util
import sys

import pytest
import yaml


_ROOT = Path(__file__).parents[1]
_MODULE_PATH = _ROOT / "scripts" / "heartbeat_inventory.py"
_SPEC = importlib.util.spec_from_file_location("heartbeat_inventory", _MODULE_PATH)
assert _SPEC and _SPEC.loader
inventory = importlib.util.module_from_spec(_SPEC)
sys.modules[_SPEC.name] = inventory
_SPEC.loader.exec_module(inventory)


@pytest.mark.parametrize("role", ("minipc", "jetson", "groundpc"))
def test_role_inventory_is_unambiguous_and_validated(role):
    parsed = inventory.load_inventory(_ROOT / "config" / "heartbeat" / f"{role}.yaml")
    assert parsed.role == role
    assert parsed.signals
    assert len(parsed.by_name) == len(parsed.signals)
    assert all(signal.topic.startswith("/") for signal in parsed.signals)


def test_inventory_parameters_omit_disabled_signals():
    parsed = inventory.parse_inventory(
        {
            "role": "test",
            "signals": {
                "disabled": {
                    "topic": "/disabled",
                    "type": "std_msgs/msg/Empty",
                    "timeout_sec": 1,
                    "expected_frequency_hz": 1,
                    "enabled": False,
                },
                "enabled": {
                    "topic": "/enabled",
                    "type": "std_msgs/msg/Empty",
                    "timeout_sec": 1,
                    "expected_frequency_hz": 1,
                },
            },
        }
    )
    assert [item["monitor_name"] for item in inventory.inventory_parameters(parsed)] == ["enabled"]


def test_inventory_parameters_use_configured_minimum_frequency():
    parsed = inventory.parse_inventory(
        {"role": "test", "signals": {"signal": {
            "topic": "/signal", "type": "std_msgs/msg/Empty", "timeout_sec": 1,
            "expected_frequency_hz": 20, "minimum_frequency_hz": 15,
        }}}
    )
    assert inventory.inventory_parameters(parsed)[0]["minimum_frequency"] == 15


@pytest.mark.parametrize(
    "bad",
    [
        {},
        {"role": "test", "signals": {}},
        {
            "role": "test",
            "signals": {"bad": {"topic": "relative", "type": "std_msgs/msg/Empty"}},
        },
    ],
)
def test_invalid_inventory_fails_closed(bad):
    with pytest.raises(inventory.InventoryError):
        inventory.parse_inventory(bad)


def test_legacy_minipc_config_has_one_list_key():
    with (_ROOT / "config" / "minipc_heartbeat.yaml").open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    assert isinstance(config["minipc_heartbeat"], list)
