from math import atan2, hypot, pi, sin, cos

import pytest
from mission_manager.docking_reference import docking_reference
from mission_manager.waypoint_config import Waypoint


def pose(x, y, yaw):
    return Waypoint("target", x, y, yaw, "target", "dock")


def delta(a, b):
    return atan2(sin(b-a), cos(b-a))


def test_reference_has_continuous_position_and_bow_heading_in_both_directions():
    start, goal = pose(0, 0, -2.014), pose(-1.3, -2.8, 2.9119)
    entry = docking_reference(start, goal)
    exit = docking_reference(goal, start)
    assert delta(entry[0].yaw, entry[-1].yaw) == pytest.approx(-1.357285307)
    assert entry[0].x == start.x and entry[-1].y == goal.y
    for a, b in zip(entry, entry[1:]):
        assert hypot(b.x-a.x, b.y-a.y) <= 0.05
        assert -0.03 < delta(a.yaw, b.yaw) <= 0
    for a, b in zip(entry, reversed(exit)):
        assert a.x == pytest.approx(b.x)
        assert a.y == pytest.approx(b.y)
        assert delta(a.yaw, b.yaw) == pytest.approx(0)
    assert abs(delta(entry[0].yaw, entry[1].yaw)) < abs(delta(entry[len(entry)//2].yaw, entry[len(entry)//2+1].yaw))


def test_wrap_and_pure_rotation_and_invalid_input():
    path = docking_reference(pose(0, 0, pi-0.01), pose(0, 0, -pi+0.01))
    assert sum(delta(a.yaw, b.yaw) for a, b in zip(path, path[1:])) == pytest.approx(0.02)
    with pytest.raises(ValueError):
        docking_reference(pose(0, 0, 0), pose(float('nan'), 0, 0))
    with pytest.raises(ValueError):
        docking_reference(pose(0, 0, 0), pose(1, 0, 0), spacing=0)
