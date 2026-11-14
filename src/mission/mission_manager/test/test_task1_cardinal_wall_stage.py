import ast
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NODE_SOURCE = PACKAGE_ROOT / "mission_manager" / "mission_manager_node.py"
REPO_ROOT = PACKAGE_ROOT.parents[2]


def test_wall_gate_requires_successful_gps3_not_per_goal_feedback():
    from mission_manager.mission_manager_node import MissionManager
    from mission_manager.task_registry import TaskDefinition
    from mission_manager.waypoint_config import Waypoint
    manager = object.__new__(MissionManager)
    manager._active_task = TaskDefinition("task1", "Task1", "experimental", features={"cardinal_walls": True})
    changes = []
    manager._set_task1_cardinal_walls = changes.append
    def waypoint(checkpoint):
        return Waypoint(checkpoint, 0., 0., 0., checkpoint, "waypoint", competition_id=checkpoint)
    manager._on_waypoint_succeeded(waypoint("2"))
    assert changes == []
    manager._on_waypoint_succeeded(waypoint("3"))
    assert changes == [True]


def test_coordinate_projection_has_a_bounded_failure_path():
    """A lost /fromLL response must not keep a mission active forever."""
    source = NODE_SOURCE.read_text(encoding="utf-8")
    assert 'declare_parameter("coordinate_projection_timeout_sec", 30.0)' in source
    assert "map projection service /fromLL timed out after " in source
    assert "ResultCode.CONFIGURATION_FAILED" in source
    assert "Serialize /fromLL requests" in source
    assert 'coordinate_projection_request_interval_sec' not in source
    assert 'self._request_next_coordinate_projection()' in source


def test_persistent_minipc_bringup_starts_the_task1_wall_node():
    source = (REPO_ROOT / "src" / "bringup" / "robot" / "launch" / "minipc_bringup.launch.py").read_text(
        encoding="utf-8"
    )
    assert 'executable="cardinal_wall_publisher"' in source
    assert '"wall_enable_topic": "/task1/cardinal_wall_enable"' in source
