import math

import pytest

from task2_sim.kinematics import integrate_pose


def test_integrate_pose_uses_body_frame_velocity():
    x, y, yaw = integrate_pose(0.0, 0.0, math.pi / 2.0, 2.0, 0.0, 0.5, 2.0)

    assert x == pytest.approx(0.0, abs=1e-12)
    assert y == pytest.approx(4.0)
    assert yaw == pytest.approx(math.pi / 2.0 + 1.0)
