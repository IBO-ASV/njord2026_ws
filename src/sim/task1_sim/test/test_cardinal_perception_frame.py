from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_cardinal_sim_publishes_configured_buoys_in_map_frame():
    launch_source = (PACKAGE_ROOT / "launch" / "task1_sim.launch.py").read_text(encoding="utf-8")
    node_source = (PACKAGE_ROOT / "task1_sim" / "cardinal_perception_sim.py").read_text(
        encoding="utf-8"
    )
    assert '"output_frame": "map"' in launch_source
    assert 'if self.output_frame == "map":' in node_source
    assert "detection.position.x = bx" in node_source
    assert "detection.position.y = by" in node_source
