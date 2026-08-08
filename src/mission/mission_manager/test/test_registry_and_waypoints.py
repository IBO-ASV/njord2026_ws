from pathlib import Path

import pytest

from mission_manager.task_registry import RegistryError, TaskRegistry, required_runtime_readiness
from mission_manager.waypoint_config import WaypointConfigLoader


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WAYPOINT_ROOT = PACKAGE_ROOT.parents[1] / "navigation" / "path_generator" / "waypoint_publisher"


def test_registry_exposes_supported_and_unimplemented_tasks():
    registry = TaskRegistry.from_file(
        PACKAGE_ROOT / "config" / "task_registry.yaml",
        package_shares={"waypoint_publisher": WAYPOINT_ROOT},
    )
    assert registry.get("task1").runnable
    assert registry.get("task2").executor == "task2_mppi"
    assert registry.get("task1_2").availability == "not_implemented"
    assert "Marker-driven" in registry.get("task1_2").reason


def test_existing_task_routes_load_without_changing_waypoint_files():
    loader = WaypointConfigLoader()
    task1 = loader.load(WAYPOINT_ROOT / "config/task1_waypoints.yaml", "task1_config")
    task2 = loader.load(WAYPOINT_ROOT / "config/task2_waypoints.yaml", "task2_config")
    task3 = loader.load(WAYPOINT_ROOT / "config/task3_waypoints.yaml", "task3_1_config")
    assert len(task1.waypoints) == 17
    assert [waypoint.waypoint_id for waypoint in task2.waypoints] == ["5", "gate_1", "gate_2", "6"]
    assert [waypoint.waypoint_id for waypoint in task3.stage("stage_1_gate")] == ["7", "b31_corridor_gate", "8"]


def test_duplicate_yaml_keys_are_rejected(tmp_path):
    bad = tmp_path / "duplicate.yaml"
    bad.write_text("tasks: {}\ntasks: {}\n", encoding="utf-8")
    with pytest.raises(RegistryError, match="duplicate YAML key"):
        TaskRegistry.from_file(bad)


def test_route_validation_only_satisfies_task1_requirements():
    registry = TaskRegistry.from_file(
        PACKAGE_ROOT / "config" / "task_registry.yaml",
        package_shares={"waypoint_publisher": WAYPOINT_ROOT},
    )
    assert required_runtime_readiness(registry.get("task1")) == frozenset()
    assert required_runtime_readiness(registry.get("task2")) == {
        "collision_monitor", "buoy_perception"
    }
    assert required_runtime_readiness(registry.get("task3_1")) == {
        "collision_monitor", "dynamic_gate_tf"
    }
