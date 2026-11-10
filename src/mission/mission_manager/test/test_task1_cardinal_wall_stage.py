import ast
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NODE_SOURCE = PACKAGE_ROOT / "mission_manager" / "mission_manager_node.py"
REPO_ROOT = PACKAGE_ROOT.parents[2]


def test_mission_manager_latches_cardinal_walls_from_nav2_feedback():
    """Keep the normal Mission Manager path wired to the GPS3 wall gate."""
    source = NODE_SOURCE.read_text(encoding="utf-8")
    ast.parse(source)
    assert '"/task1/cardinal_wall_enable"' in source
    assert '"/task1/gps3_to_gps4_heading"' in source
    assert "number_of_poses_remaining" in source
    assert "competition_id" in source


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
    source = (REPO_ROOT / "src" / "robot" / "launch" / "minipc_bringup.launch.py").read_text(
        encoding="utf-8"
    )
    assert 'executable="cardinal_wall_publisher"' in source
    assert '"wall_enable_topic": "/task1/cardinal_wall_enable"' in source
