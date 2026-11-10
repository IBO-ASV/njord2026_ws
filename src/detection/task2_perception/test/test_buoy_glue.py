import numpy as np

from task2_perception import buoy_glue


def test_route_side_assigns_left_red_and_right_green():
    start, end = np.array([0.0, 0.0]), np.array([20.0, 0.0])
    assert buoy_glue.classify_route_side(
        np.array([10.0, 2.5]), start, end, 2.5, 0.5) == buoy_glue.RED
    assert buoy_glue.classify_route_side(
        np.array([10.0, -2.5]), start, end, 2.5, 0.5) == buoy_glue.GREEN


def test_route_side_rejects_wrong_offset_or_outside_route():
    start, end = np.array([0.0, 0.0]), np.array([20.0, 0.0])
    assert buoy_glue.classify_route_side(
        np.array([10.0, 0.0]), start, end, 2.5, 0.5) is None
    assert buoy_glue.classify_route_side(
        np.array([21.0, -2.5]), start, end, 2.5, 0.5) is None


def test_route_side_accepts_three_metre_band_centred_at_two_point_five_m():
    start, end = np.array([0.0, 0.0]), np.array([20.0, 0.0])
    assert buoy_glue.classify_route_side(
        np.array([10.0, 3.9]), start, end, 2.5, 1.5) == buoy_glue.RED
    assert buoy_glue.classify_route_side(
        np.array([10.0, -3.9]), start, end, 2.5, 1.5) == buoy_glue.GREEN
    assert buoy_glue.classify_route_side(
        np.array([10.0, 0.9]), start, end, 2.5, 1.5) is None
    assert buoy_glue.classify_route_side(
        np.array([10.0, 4.1]), start, end, 2.5, 1.5) is None
    assert buoy_glue.classify_route_side(
        np.array([-0.1, 2.5]), start, end, 2.5, 1.5) is None


def test_stationary_gate_uses_compensated_ground_speed():
    assert buoy_glue.is_stationary(np.array([0.2, -0.1, 0.0]), 0.25)
    assert not buoy_glue.is_stationary(np.array([0.3, 0.2, 0.0]), 0.25)
