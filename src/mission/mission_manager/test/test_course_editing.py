import copy

import pytest
import yaml
from mission_manager.course_editing import (
    atomic_save,
    revision,
    validate_edit,
    waypoint_yaml_edit,
)


def test_yaml_edit_preserves_comments_aliases_and_mission_metadata(tmp_path):
    text = """# surveyed route; keep this comment
route:
  waypoints: &points
    - {id: 1, latitude: 34.5, longitude: 135.5, yaw: 1.2, type: start, name: GPS1}
    - {id: 2, latitude: 34.6, longitude: 135.6, yaw: 1.3, type: goal, name: GPS2}
  stages: {transit: [1, 2]}
compatibility:
  waypoints: *points
"""
    points = [
        {"id": "1", "latitude": 34.51, "longitude": 135.51},
        {"id": "2", "latitude": 34.61, "longitude": 135.61},
    ]
    buoys = [{"id": "b1", "latitude": 34.55, "longitude": 135.55, "type": "N"}]
    changed = waypoint_yaml_edit(text, "route", points, buoys)
    assert (
        "# surveyed route" in changed and "&points" in changed and "*points" in changed
    )
    data = yaml.safe_load(changed)
    assert data["route"]["waypoints"][0]["latitude"] == 34.51
    assert data["route"]["waypoints"][0]["yaw"] == 1.2
    assert data["route"]["waypoints"][0]["type"] == "start"
    assert data["compatibility"]["waypoints"][0]["latitude"] == 34.51
    assert data["route"]["stages"] == {"transit": [1, 2]}
    assert data["map_buoys"]["route"] == buoys
    # Repeated saves replace the buoy block rather than accumulating keys.
    changed = waypoint_yaml_edit(changed, "route", points, [])
    assert yaml.safe_load(changed)["map_buoys"]["route"] == []
    source = tmp_path / "route.yaml"
    source.write_text(text)
    alias = tmp_path / "installed.yaml"
    alias.symlink_to(source)
    token = revision(source)
    atomic_save(alias, changed, token)
    assert alias.is_symlink() and source.read_text() == changed
    with pytest.raises(ValueError, match="changed externally"):
        atomic_save(source, text, token)
    assert source.read_text() == changed


@pytest.mark.parametrize(
    "mutation", ["nan", "range", "ids", "type", "duplicate", "mapping"]
)
def test_reject_invalid_course_edits(mutation):
    original = {"waypoints": [{"id": "1", "latitude": 34.0, "longitude": 135.0}]}
    edit = copy.deepcopy(original)
    edit["buoys"] = [{"id": "b", "latitude": 34.0, "longitude": 135.0, "type": "RED"}]
    if mutation == "nan":
        edit["waypoints"][0]["latitude"] = float("nan")
    if mutation == "range":
        edit["waypoints"][0]["latitude"] = 91
    if mutation == "ids":
        edit["waypoints"][0]["id"] = "other"
    if mutation == "type":
        edit["buoys"][0]["type"] = "unknown"
    if mutation == "duplicate":
        edit["buoys"].append(dict(edit["buoys"][0]))
    if mutation == "mapping":
        edit["buoys"] = ["bad"]
    with pytest.raises(ValueError):
        validate_edit(original, edit)
