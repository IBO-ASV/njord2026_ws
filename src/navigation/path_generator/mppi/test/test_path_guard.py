from types import SimpleNamespace

from asv_trajectory_planner.path_guard import guarded_path, lane_path, safe_path


def test_head_on_path_is_rejected_and_replaced_on_starboard_side():
    other = SimpleNamespace(x=35.0, y=0.0, vx=-1.286, vy=0.0)
    buoys = [(5.0, 2.5), (5.0, -2.5), (30.0, 2.5), (30.0, -2.5)]
    raw = [(i * 0.25, 0.0) for i in range(145)]
    assert not safe_path(raw, other, 1.03, buoys, (0.0, 0.0), (36.0, 0.0))
    path, message = guarded_path(raw, (0.0, 0.0), (36.0, 0.0), other, 1.03, buoys)
    assert path is not None, message
    assert min(y for _, y in path) < -3.0
    assert path[-1] == (36.0, 0.0)
    assert safe_path(path, other, 1.03, buoys, (0.0, 0.0), (36.0, 0.0))


def test_observed_gate_bounds_prevent_a_detour_outside_the_gap():
    buoys = [(5.0, 2.5), (5.0, -2.5)]
    assert not safe_path(
        [(0.0, 0.0), (5.0, -4.0), (10.0, 0.0)],
        None,
        1.0,
        buoys,
        (0.0, 0.0),
        (10.0, 0.0),
    )
    path = lane_path((0.0, 0.0), (36.0, 0.0), -5.0, buoys)
    near = min(path, key=lambda p: abs(p[0] - 5.0))
    assert abs(near[1]) < 0.1


def test_close_obstacle_without_a_safe_candidate_inhibits_the_path():
    path, _ = guarded_path(
        [(0.0, 0.0), (10.0, 0.0)], (0.0, 0.0), (10.0, 0.0), None, 1.0, [(0.0, 0.0)]
    )
    assert path is None


def test_small_head_on_tracking_noise_does_not_choose_a_port_lane():
    other = SimpleNamespace(x=100.0, y=1.0, vx=-1.0, vy=-0.4)
    path, _ = guarded_path(
        [(0.0, 0.0), (36.0, 0.0)], (0.0, 0.0), (36.0, 0.0), other, 1.0, []
    )
    assert path is not None
    assert min(y for _, y in path) < -4.0
    assert max(y for _, y in path) <= 0.0


def test_committed_encounter_lane_survives_later_classification_noise():
    other = SimpleNamespace(x=100.0, y=-1.0, vx=-1.0, vy=0.0)
    path, _ = guarded_path(
        [(0.0, 0.0), (36.0, 0.0)],
        (0.0, 0.0),
        (36.0, 0.0),
        other,
        1.0,
        [],
        preferred_lane=5.0,
    )
    assert path is not None
    assert max(y for _, y in path) > 4.0
    assert min(y for _, y in path) >= 0.0


def test_delayed_head_on_approach_uses_a_wider_safe_lane_before_holding():
    # Recorded noisy seed 2027 observation, rotated into GPS-course axes.
    # The boat has not yet cleared the first gate as the target approaches.
    start, end = (-4.423380, 0.107910), (31.686909, 0.107910)
    buoys = [
        (26.309013, 2.648658),
        (26.307643, -2.395348),
        (0.935927, 2.618822),
        (1.025608, -2.414124),
    ]
    other = SimpleNamespace(x=26.645441, y=0.060219, vx=-1.335715, vy=-0.183184)
    for lane in [-5.0, -3.5]:
        assert not safe_path(
            lane_path(start, end, lane, buoys), other, 1.0288888889, buoys, start, end
        )
    path, message = guarded_path(
        [], start, end, other, 1.0288888889, buoys, preferred_lane=-5.0
    )
    assert path is not None, message
    assert min(y for _, y in path) < -6.0
    assert max(y for _, y in path) <= 0.107910
    assert safe_path(path, other, 1.0288888889, buoys, start, end)


def test_gate_approaches_are_straight_without_an_initial_lane_excursion():
    buoys = [(5.0, 2.5), (5.0, -2.5), (30.0, 2.5), (30.0, -2.5)]
    path = lane_path((0.0, 0.0), (36.0, 0.0), -7.0, buoys)
    assert all(abs(y) < 1e-9 for x, y in path if x <= 6.0 or x >= 29.0)
    assert min(y for _, y in path) < -6.0


def test_replan_near_a_gate_does_not_jump_from_measured_lateral_position():
    # Ownship is 1 m to starboard, 2 m beyond the first gate.
    buoys = [(-2.0, 3.5), (-2.0, -1.5), (23.0, 3.5), (23.0, -1.5)]
    path = lane_path((-7.0, 1.0), (29.0, 1.0), -5.0, buoys, spacing=0.01)
    assert path[0] == (0.0, 0.0)
    assert abs(path[1][1]) < 0.02
