"""Timed stale-target integration checks for Task2's path guard."""

from types import SimpleNamespace

from asv_trajectory_planner.path_guard import safe_path

import pytest


def test_safe_path_rejects_inside_clearance_and_accepts_exact_boundary():
    """Keep the strict clearance boundary behavior covered."""
    path = [(0.0, 0.0), (1.0, 0.0)]
    start, end = (0.0, 0.0), (10.0, 0.0)

    def other_at(y):
        return SimpleNamespace(x=0.0, y=y, vx=0.0, vy=0.0)

    assert not safe_path(path, other_at(2.849), 1.0, [], start, end)
    assert safe_path(path, other_at(2.85), 1.0, [], start, end)
    assert safe_path(path, other_at(2.851), 1.0, [], start, end)


def test_stale_crossing_target_holds_until_five_meters_behind(monkeypatch):
    """Exercise the timestamped guard as a crossing target goes stale."""
    pytest.importorskip('geometry_msgs')
    pytest.importorskip('nav_msgs')

    import asv_trajectory_planner.trajectory_generator as trajectory_module
    from asv_trajectory_planner.trajectory_generator import TrajectoryGenerator
    from geometry_msgs.msg import PoseStamped, Transform, TwistStamped
    from nav_msgs.msg import Odometry

    now = [100.0]
    monkeypatch.setattr(trajectory_module.time, 'monotonic', lambda: now[0])
    generator = TrajectoryGenerator(
        path_guard_enabled=True,
        other_twist_is_relative=False,
        opponent_use_distance_m=50.0,
    )
    # Keep real state conversion and guard logic, with deterministic planner
    # candidates independent of GPU/MPPI sampling.
    generator.planner.plan = lambda **_kwargs: [
        (index * 0.25, 0.0) for index in range(145)
    ]

    def ownship(x):
        odom = Odometry()
        odom.pose.pose.position.x = x
        odom.pose.pose.orientation.w = 1.0
        return odom

    def waypoint(x):
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.orientation.w = 1.0
        return pose

    start, goal = waypoint(0.0), waypoint(36.0)
    crossing = Transform()
    crossing.translation.x = 0.0
    crossing.translation.y = 5.0
    crossing.rotation.w = 1.0
    crossing_twist = TwistStamped()
    crossing_twist.twist.linear.y = -1.3

    # Record a port-to-starboard crossing target in map coordinates.
    generator.generate(ownship(0.0), crossing, crossing_twist, start, goal)
    assert generator._guard_target is not None

    # After 4.1 s, predicted separation is 2.03 m: inside the 2.85 m radius.
    now[0] = 104.1
    generator.generate(ownship(2.0), None, None, start, goal)
    assert not generator.path_guard_safe
    assert generator.path_guard_message == (
        'opponent observation stale before safe passage'
    )
    assert generator._guard_target is not None

    # At a predicted 3.04 m, it has left the radius but remains inside the 5 m rear buffer.
    now[0] = 104.2
    generator.generate(ownship(3.0), None, None, start, goal)
    assert not generator.path_guard_safe
    assert generator.path_guard_message == (
        'opponent observation stale before safe passage'
    )
    assert generator._guard_target is not None

    # The release comparison is strict: exactly 5 m behind still holds.
    now[0] = 105.0
    generator.generate(ownship(5.0), None, None, start, goal)
    assert not generator.path_guard_safe
    assert generator._guard_target is not None

    # A further 1 cm in the prediction releases the target and permits a path.
    now[0] = 105.01
    generator.generate(ownship(5.01), None, None, start, goal)
    assert generator.path_guard_safe
    assert generator.path_guard_message == 'MPPI path accepted'
    assert generator._guard_target is None
