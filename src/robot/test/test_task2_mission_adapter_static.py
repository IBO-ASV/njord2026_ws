from pathlib import Path


def test_task2_mission_adapter_keeps_persistent_owners_and_uses_mission_gate():
    text = (Path(__file__).parents[1] / "launch" / "task2_mission_adapter.launch.py").read_text()

    assert '"task2_perception.launch.py"' in text
    assert '"classical_pipeline.launch.py"' in text
    assert '"planner_real.launch.py"' in text
    assert "/mission/task2/enabled" in text
    assert '"mission_gate_required": "true"' in text
    for forbidden in (
        "thruster_driver",
        "command_arbiter_node",
        "navigation_launch_task2.py",
        "task2_autonomy_ready",
    ):
        assert forbidden not in text


def test_minipc_starts_task2_adapter_only_for_task2_profile():
    text = (Path(__file__).parents[1] / "launch" / "minipc_bringup.launch.py").read_text()

    assert '"task2_mission_adapter.launch.py"' in text
    assert "active_nav2_profile" in text
    assert "' == 'task2'" in text
