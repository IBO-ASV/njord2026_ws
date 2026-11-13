import math
from njord_sim.scenario import (
    rectangle,
    segment_hits_polygon,
    point_rectangle_clearance,
    inverse_enu,
    dock_segments,
)
from mission_manager.geodesy import HomeDatum, wgs84_to_enu


def test_projection_round_trip_preserves_survey_metres():
    datum = HomeDatum(63.4408, 10.4233194444, 0.0)
    for x, y in ((-27.099, -29.685), (23.089, -3.401), (0.0, 0.0)):
        lat, lon = inverse_enu(x, y, datum)
        actual = wgs84_to_enu(lat, lon, datum)
        assert math.hypot(actual[0] - x, actual[1] - y) < 1e-5


def test_hull_side_contact_is_detected_when_centre_is_clear():
    hull = rectangle(0.0, 0.0, math.pi / 4)
    assert segment_hits_polygon((0.6, -2.0), (0.6, 2.0), hull)
    assert not segment_hits_polygon((2.0, -2.0), (2.0, 2.0), hull)
    assert segment_hits_polygon((0.0, 0.0), (0.1, 0.0), hull)


def test_buoy_contact_uses_oriented_hull_not_a_point_or_large_disc():
    assert point_rectangle_clearance(0.8, 0.0, 0.0, 0.0, 0.0) < 0
    assert point_rectangle_clearance(0.0, 0.8, 0.0, 0.0, 0.0) > 0
    assert point_rectangle_clearance(0.0, 0.8, 0.0, 0.0, math.pi / 2) < 0


def test_parallel_berth_can_open_on_its_long_side():
    # A sideways approach must cross the mouth, not an invented closed wall.
    dock = (0.0, 0.0, math.pi / 2, 4.0, 2.0, 0)
    walls = dock_segments(dock)
    approach_hull = rectangle(1.0, 0.0, math.pi / 2)
    assert not any(segment_hits_polygon(a, b, approach_hull) for a, b in walls)
    opposite_hull = rectangle(-1.0, 0.0, math.pi / 2)
    assert any(segment_hits_polygon(a, b, opposite_hull) for a, b in walls)


def test_task3_2_reference_rotates_through_open_mouth_without_hull_contact():
    from mission_manager.docking_reference import docking_reference
    from njord_sim.scenario import scenario

    s = scenario("task3_2")
    approach, berth = s.route.waypoints[1:3]
    direction = math.atan2(berth.y-approach.y, berth.x-approach.x)
    assert abs(math.atan2(math.sin(approach.yaw-direction), math.cos(approach.yaw-direction))) < 0.002
    rotation = math.atan2(math.sin(berth.yaw-approach.yaw), math.cos(berth.yaw-approach.yaw))
    assert -math.pi/2 < rotation < -math.pi/3
    assert s.route.constraints["se2_docking_stages"] == ["stage_2", "stage_3"]
    for start, goal in ((approach, berth), (berth, approach)):
        for p in docking_reference(start, goal):
            assert not any(segment_hits_polygon(a, b, rectangle(p.x, p.y, p.yaw))
                           for dock in s.docks for a, b in dock_segments(dock))
    assert "se2_docking_stages" not in scenario("task3_1").route.constraints
