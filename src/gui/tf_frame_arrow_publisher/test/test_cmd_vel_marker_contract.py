from pathlib import Path


def test_cmd_vel_marker_uses_base_link_arrow_and_ten_meter_circle():
    source = (Path(__file__).resolve().parents[1] / "src" / "cmd_vel_marker_publisher.cpp").read_text()
    assert '"/cmd_vel"' in source
    assert '"/cmd_vel_markers"' in source
    assert '"base_link"' in source
    assert "kRadius = 10.0" in source
    assert "kMotionEpsilon" in source
    assert "Marker::DELETE" in source
    assert "Marker::ARROW" in source
    assert "Marker::LINE_STRIP" in source
