from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_task2_sim_runtime_is_mission_managed_and_resets_per_start():
    manager = (ROOT / "mission_manager" / "runtime_manager_node.py").read_text()
    mission = (ROOT / "mission_manager" / "mission_manager_node.py").read_text()
    assert '"task2_sim"' in manager
    assert '"mission_managed:=true"' in manager
    assert "restart_on_configure" in manager
    assert "force_runtime_reconfigure" in mission
    assert "skip_coordinate_projection" in mission
