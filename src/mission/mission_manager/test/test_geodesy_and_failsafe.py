from math import pi

from mission_manager.geodesy import HomeDatum, wgs84_to_enu
from mission_manager.ground_link_failsafe import GroundLinkReturnEvaluator
from mission_manager.waypoint_config import WaypointConfigLoader


HOME = HomeDatum(63.44066402076241, 10.423546644308498, pi / 2)


def test_home_is_enu_origin_and_geographic_waypoint_loads(tmp_path):
    east, north = wgs84_to_enu(HOME.latitude, HOME.longitude, HOME)
    assert abs(east) < 1e-6
    assert abs(north) < 1e-6
    route = tmp_path / "route.yaml"
    route.write_text(
        "route:\n  frame_id: map\n  waypoints:\n"
        f"    - id: home\n      latitude: {HOME.latitude}\n      longitude: {HOME.longitude}\n"
        "      yaw: 1.5707963267948966\n",
        encoding="utf-8",
    )
    loaded = WaypointConfigLoader(HOME).load(route, "route")
    assert abs(loaded.waypoints[0].x) < 1e-6
    assert abs(loaded.waypoints[0].y) < 1e-6


def test_ground_link_failsafe_triggers_after_stable_twenty_to_thirty_second_window():
    evaluator = GroundLinkReturnEvaluator(HOME)
    evaluator.heartbeat(0.0)
    for second in range(20, 31):
        evaluator.position(float(second), HOME.latitude, HOME.longitude)
        evaluator.heading(float(second), pi / 2)
    decision = evaluator.evaluate(30.0)
    assert decision is not None
    assert decision.trigger_return_home
    assert evaluator.evaluate(31.0) is None


def test_ground_link_failsafe_refuses_unstable_position():
    evaluator = GroundLinkReturnEvaluator(HOME)
    evaluator.heartbeat(0.0)
    for second in range(20, 31):
        # About 2.2 m north/south alternation exceeds the 1 m radial standard deviation.
        latitude = HOME.latitude + (0.00002 if second % 2 else -0.00002)
        evaluator.position(float(second), latitude, HOME.longitude)
        evaluator.heading(float(second), pi / 2)
    decision = evaluator.evaluate(30.0)
    assert decision is not None
    assert not decision.trigger_return_home
    assert "position stddev" in decision.status


def test_ground_link_failsafe_resets_when_any_keepalive_arrives():
    evaluator = GroundLinkReturnEvaluator(HOME)
    evaluator.heartbeat(0.0)
    evaluator.heartbeat(25.0)
    decision = evaluator.evaluate(30.0)
    assert decision is not None
    assert not decision.trigger_return_home
    assert "waiting" in decision.status
