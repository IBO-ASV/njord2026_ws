from pathlib import Path


def test_task2_autonomy_launch_wires_mppi_velocity_through_arbiter():
    text = (Path(__file__).parents[1] / "launch" / "task2_autonomy.launch.py").read_text()

    assert '"planner_real.launch.py"' in text
    assert '"navigation_launch_task2.py"' in text
    assert '"auto_cmd_vel_topic": "/cmd_vel_nav"' in text
    assert 'executable="task2_autonomy_ready_node"' in text
    assert '"opponent_motion_mode", default_value="straight_line"' in text
    assert '"motion_filter_mode": opponent_motion_mode' in text
    assert '"motion_mode": opponent_motion_mode' in text
    assert text.count('"task2_params.yaml"') == 3
    assert '"path_topic": "/planned_path_pruned"' in text
    assert "manual_control.launch.py" in text
    assert "command_arbiter_node" not in text
    assert "real_bringup.launch.py" not in text


def test_task2_nav2_launch_is_follow_path_only():
    text = (Path(__file__).parents[1] / "launch" / "navigation_launch_task2.py").read_text()

    assert "lifecycle_nodes = ['controller_server', 'velocity_smoother']" in text
    for unused_node in (
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
    ):
        assert f"name='{unused_node}'" not in text
