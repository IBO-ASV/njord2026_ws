"""Tests 6-7: ego compensation and stale-track invalidation (numpy only)."""

import numpy as np

from task2_perception import tracking_glue
from task2_perception.cloud_ops import make_transform, rpy_to_rotation_matrix
from task2_perception.smoothing import TwistSmoother, knots_to_mps, mps_to_knots
from task2_perception.tracking_glue import SelectionParams, Track


def make_track(**kw):
    defaults = dict(
        object_id=1,
        position=np.array([20.0, 5.0, 0.0]),
        yaw=0.0,
        velocity_body=np.array([-1.0, 0.5, 0.0]),
        yaw_rate=0.0,
        dimensions=np.array([6.0, 2.0, 1.5]),
        point_count=50,
        track_state=tracking_glue.TRACK_CONFIRMED,
        stamp_sec=100.0,
    )
    defaults.update(kw)
    return Track(**defaults)


class TestEgoCompensation:
    """Test 6: relative velocity + ego velocity -> ground velocity (with rotation)."""

    def test_body_frame_twist_rotates_into_base_axes(self):
        # Object heading +90 deg in base_link, moving 1 m/s along its own x.
        track = make_track(yaw=np.pi / 2.0, velocity_body=np.array([1.0, 0.0, 0.0]))
        np.testing.assert_allclose(track.velocity_base, [0.0, 1.0, 0.0], atol=1e-12)

    def test_yaw_rotated_compensation(self):
        # Own ship heading 90 deg in map, moving 2 m/s along its own x.
        yaw = np.pi / 2.0
        t_map_base = make_transform(
            rpy_to_rotation_matrix(0.0, 0.0, yaw), [100.0, 50.0, 0.0])
        ego_vel_base = np.array([2.0, 0.0, 0.0])
        # Opponent seen closing at -1 m/s along own x, +0.5 along own y
        # (object yaw = 0 -> body frame == base axes).
        track = make_track(position=np.array([10.0, 0.0, 0.0]))
        vel_abs_base = tracking_glue.ego_compensate(track.velocity_base, ego_vel_base)
        np.testing.assert_allclose(vel_abs_base, [1.0, 0.5, 0.0], atol=1e-12)
        pos_map, vel_map = tracking_glue.to_map_frame(
            track.position, vel_abs_base, t_map_base)
        # Rz(90): (1, 0.5) -> (-0.5, 1.0); (10, 0) -> (0, 10) + (100, 50).
        np.testing.assert_allclose(vel_map, [-0.5, 1.0, 0.0], atol=1e-12)
        np.testing.assert_allclose(pos_map, [100.0, 60.0, 0.0], atol=1e-12)

    def test_yaw_rate_restores_omega_cross_r_term(self):
        # Own ship spinning at wz=0.2 rad/s with zero linear velocity; a
        # physically stationary target at r=(0, 20, 0) appears to move at
        # -omega x r = -(-4, 0, 0) = (+4, 0, 0) in the rotating base_link
        # frame. Compensation adds omega x r back, restoring zero ground
        # velocity.
        ego_vel_base = np.zeros(3)
        ego_wz = 0.2
        pos = np.array([0.0, 20.0, 0.0])
        apparent = np.array([4.0, 0.0, 0.0])
        track = make_track(position=pos, velocity_body=apparent)
        vel_abs = tracking_glue.ego_compensate(
            track.velocity_base, ego_vel_base,
            ego_yaw_rate=ego_wz, pos_base=pos)
        # omega x r = (-0.2*20, 0.2*0, 0) = (-4, 0, 0); 4 + (-4) = 0.
        np.testing.assert_allclose(vel_abs, np.zeros(3), atol=1e-12)

    def test_yaw_rate_ignored_without_position(self):
        # Backwards-compatible: no pos_base -> pure linear compensation.
        v = tracking_glue.ego_compensate(
            np.array([1.0, 0.0, 0.0]), np.array([0.5, 0.0, 0.0]),
            ego_yaw_rate=0.3, pos_base=None)
        np.testing.assert_allclose(v, [1.5, 0.0, 0.0], atol=1e-12)

    def test_stationary_target_gets_zero_ground_speed(self):
        # A physically stationary target appears to move at -v_ego relative to
        # us; after compensation its ground velocity must vanish.
        ego_vel_base = np.array([1.5, -0.2, 0.0])
        track = make_track(velocity_body=-ego_vel_base)
        vel_abs = tracking_glue.ego_compensate(track.velocity_base, ego_vel_base)
        np.testing.assert_allclose(vel_abs, np.zeros(3), atol=1e-12)
        t_map_base = make_transform(
            rpy_to_rotation_matrix(0.0, 0.0, 0.7), [3.0, 4.0, 0.0])
        _, vel_map = tracking_glue.to_map_frame(track.position, vel_abs, t_map_base)
        np.testing.assert_allclose(vel_map, np.zeros(3), atol=1e-12)

    def test_short_coast_uses_constant_ground_velocity(self):
        position = np.array([10.0, -2.0, 0.0])
        velocity = np.array([1.0, 0.5, 0.0])
        predicted = tracking_glue.predict_straight_motion(position, velocity, 1.5)
        np.testing.assert_allclose(predicted, [11.5, -1.25, 0.0], atol=1e-12)


class TestStaleAndGates:
    """Test 7: stale tracks are invalidated after stale_timeout_sec."""

    def test_fresh_then_stale(self):
        params = SelectionParams(stale_timeout_sec=2.0)
        track = make_track(stamp_sec=100.0)
        assert tracking_glue.select_opponent([track], now_sec=101.0, params=params)
        assert tracking_glue.select_opponent(
            [track], now_sec=102.5, params=params) == []
        assert any(r == "stale" for _, r in params.rejected)

    def test_gates(self):
        confirmed = make_track(object_id=1, position=np.array([20.0, 0.0, 0.0]))
        tentative = make_track(object_id=2, track_state=tracking_glue.TRACK_TENTATIVE,
                               position=np.array([5.0, 0.0, 0.0]))
        tiny = make_track(object_id=3, dimensions=np.array([0.3, 0.3, 0.3]),
                          position=np.array([8.0, 0.0, 0.0]))
        sparse = make_track(object_id=4, point_count=3,
                            position=np.array([9.0, 0.0, 0.0]))
        far = make_track(object_id=5, position=np.array([200.0, 0.0, 0.0]))
        params = SelectionParams(min_point_count=5, min_length_m=0.5,
                                 max_length_m=30.0, max_distance_m=60.0)
        ranked = tracking_glue.select_opponent(
            [confirmed, tentative, tiny, sparse, far],
            now_sec=100.5, params=params)
        assert [t.object_id for t in ranked] == [1]
        assert len(params.rejected) == 4

    def test_width_gate(self):
        narrow = make_track(dimensions=np.array([1.8, 0.3, 1.2]))
        wide = make_track(dimensions=np.array([1.8, 1.6, 1.2]))
        valid = make_track(object_id=3, dimensions=np.array([1.8, 0.9, 1.2]))
        params = SelectionParams(
            min_length_m=1.2, max_length_m=2.5,
            min_width_m=0.5, max_width_m=1.5)
        ranked = tracking_glue.select_opponent(
            [narrow, wide, valid], now_sec=100.5, params=params)
        assert [track.object_id for track in ranked] == [3]
        assert [reason for _, reason in params.rejected] == ["too narrow", "too wide"]

    def test_oriented_corridor_is_fixed_to_gps5_to_gps6_line(self):
        # GPS5->6 is east here; the vessel's own heading is irrelevant.
        start = np.array([10.0, 20.0])
        end = np.array([30.0, 20.0])
        assert tracking_glue.in_oriented_corridor(
            np.array([15.0, 40.0]), start, end, 5.0, 5.0, 20.0)
        assert not tracking_glue.in_oriented_corridor(
            np.array([14.9, 20.0]), start, end, 5.0, 5.0, 20.0)
        assert not tracking_glue.in_oriented_corridor(
            np.array([35.1, 20.0]), start, end, 5.0, 5.0, 20.0)
        assert not tracking_glue.in_oriented_corridor(
            np.array([20.0, 40.1]), start, end, 5.0, 5.0, 20.0)

    def test_route_left_side_is_fixed_to_gps5_to_gps6_line(self):
        start = np.array([10.0, 20.0])
        end = np.array([30.0, 20.0])
        assert tracking_glue.is_left_of_oriented_line(
            np.array([20.0, 20.1]), start, end)
        assert not tracking_glue.is_left_of_oriented_line(
            np.array([20.0, 20.0]), start, end)
        assert not tracking_glue.is_left_of_oriented_line(
            np.array([20.0, 19.9]), start, end)

    def test_velocity_bearing_clip_keeps_speed_on_right_to_left_sector(self):
        velocity = np.array([10.0, 0.0, 0.2])
        clipped = tracking_glue.clip_velocity_bearing(
            velocity, 0.0, np.pi / 2.0, np.pi)
        np.testing.assert_allclose(clipped, [0.0, 10.0, 0.2], atol=1e-12)
        np.testing.assert_allclose(velocity, [10.0, 0.0, 0.2], atol=1e-12)

    def test_velocity_bearing_clip_handles_zero_across_pi_wrap(self):
        route_yaw = np.deg2rad(179.0)
        velocity = 10.0 * np.array([
            np.cos(np.deg2rad(-179.0)), np.sin(np.deg2rad(-179.0)), 0.0])
        clipped = tracking_glue.clip_velocity_bearing(
            velocity, route_yaw, np.pi / 2.0, np.pi)
        np.testing.assert_allclose(
            clipped[:2], 10.0 * np.array([
                np.cos(route_yaw + np.pi / 2.0),
                np.sin(route_yaw + np.pi / 2.0)]),
            atol=1e-12)

    def test_velocity_bearing_clip_leaves_zero_speed_unchanged(self):
        velocity = np.array([0.0, 0.0, 0.2])
        clipped = tracking_glue.clip_velocity_bearing(
            velocity, 0.0, np.pi / 2.0, np.pi)
        np.testing.assert_allclose(clipped, velocity, atol=1e-12)

    def test_straight_line_confidence_gate(self):
        short_history = make_track(
            object_id=1, hit_count=8, velocity_stddev_mps=0.10)
        uncertain = make_track(
            object_id=2, hit_count=15, velocity_stddev_mps=0.40)
        settled = make_track(
            object_id=3, hit_count=15, velocity_stddev_mps=0.10)
        params = SelectionParams(
            min_hit_count=15, max_velocity_stddev_mps=0.30)
        ranked = tracking_glue.select_opponent(
            [short_history, uncertain, settled], now_sec=100.5, params=params)
        assert [track.object_id for track in ranked] == [3]
        assert [reason for _, reason in params.rejected] == [
            "insufficient observation history", "velocity estimate too uncertain"]

    def test_policies(self):
        near_diverging = make_track(object_id=1,
                                    position=np.array([10.0, 0.0, 0.0]),
                                    velocity_body=np.array([2.0, 0.0, 0.0]))
        far_colliding = make_track(object_id=2,
                                   position=np.array([30.0, 0.0, 0.0]),
                                   velocity_body=np.array([-2.0, 0.0, 0.0]))
        tracks = [near_diverging, far_colliding]
        nearest = tracking_glue.select_opponent(tracks, 100.5, policy="nearest")
        assert nearest[0].object_id == 1
        min_tcpa = tracking_glue.select_opponent(tracks, 100.5, policy="min_tcpa")
        assert min_tcpa[0].object_id == 2  # approaching ranks before diverging
        by_id = tracking_glue.select_opponent(
            tracks, 100.5, policy="track_id",
            params=SelectionParams(target_track_id=2))
        assert [t.object_id for t in by_id] == [2]
        missing = tracking_glue.select_opponent(
            tracks, 100.5, policy="track_id",
            params=SelectionParams(target_track_id=99))
        assert missing == []

    def test_cpa_tcpa(self):
        # Head-on: target at (100, 0) closing at -10 m/s -> TCPA 10 s, CPA 0 m.
        cpa, tcpa = tracking_glue.compute_cpa_tcpa(
            np.zeros(3), np.zeros(3), [100.0, 0.0, 0.0], [-10.0, 0.0, 0.0])
        assert abs(tcpa - 10.0) < 1e-9
        assert abs(cpa) < 1e-9
        # Diverging: tcpa < 0, CPA evaluated at t=0 (current distance).
        cpa, tcpa = tracking_glue.compute_cpa_tcpa(
            np.zeros(3), np.zeros(3), [100.0, 0.0, 0.0], [10.0, 0.0, 0.0])
        assert tcpa < 0.0
        assert abs(cpa - 100.0) < 1e-9


class TestTwistSmoother:

    def test_knot_conversion_round_trip(self):
        assert knots_to_mps(2.0) == 1852.0 * 2.0 / 3600.0
        assert mps_to_knots(knots_to_mps(3.0)) == 3.0

    def test_lowpass_and_spike_rejection(self):
        sm = TwistSmoother(alpha=0.5, max_speed_mps=5.0)
        first = sm.update(1.0, 0.0)
        np.testing.assert_allclose(first, (1.0, 0.0, 0.0))
        # Speed above max_speed_mps is rejected (None) and does not pollute.
        assert sm.update(10.0, 0.0) is None
        # Normal update is low-passed against the untouched state.
        second = sm.update(2.0, 0.0)
        np.testing.assert_allclose(second, (1.5, 0.0, 0.0))

    def test_reset(self):
        sm = TwistSmoother(alpha=0.5, max_speed_mps=5.0)
        sm.update(1.0, 0.0)
        sm.reset()
        np.testing.assert_allclose(sm.update(3.0, 0.0), (3.0, 0.0, 0.0))
