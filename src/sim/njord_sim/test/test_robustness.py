"""Synthetic motion must be continuous and keep truth during recognition loss."""

import math

import pytest
from njord_sim.robustness import detection_missing, opponent_motion
from njord_sim.telemetry import TrajectorySampler


def test_stationary_jitter_is_suppressed_but_turns_and_motion_are_kept():
    sampler = TrajectorySampler()
    assert sampler.accept(0.0, 0.0, math.pi - 0.01)
    assert not sampler.accept(0.02, -0.02, -math.pi + 0.01)
    assert sampler.accept(0.11, 0.0, -math.pi + 0.01)
    assert sampler.accept(0.11, 0.0, -math.pi + 0.1)


@pytest.mark.parametrize("t", [0.0, 7.5, 15.0, 22.5, 30.0, 61.0])
def test_variable_speed_matches_position_derivative(t):
    distance, speed = opponent_motion(t, 30, True)
    epsilon = 1e-4
    after, _ = opponent_motion(t + epsilon, 30, True)
    before, _ = opponent_motion(t - epsilon, 30, True)
    assert (after - before) / (2 * epsilon) == pytest.approx(speed, abs=1e-7)
    assert 0.88 < speed < 1.69
    assert math.isfinite(distance)


def test_default_motion_and_outage_boundaries():
    assert opponent_motion(35, 30) == pytest.approx((5 * 1.2861111111, 1.2861111111))
    assert not detection_missing(100, 0, 0)
    assert not detection_missing(14.99, 15, 2)
    assert detection_missing(15, 15, 2)
    assert detection_missing(16.99, 15, 2)
    assert not detection_missing(17, 15, 2)
    assert detection_missing(30, 15, 2)
