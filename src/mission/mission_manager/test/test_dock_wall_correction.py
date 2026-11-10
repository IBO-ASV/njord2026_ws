from math import cos, sin

import pytest

from mission_manager.dock_wall_correction_node import _wall_yaw_delta


def test_wall_yaw_delta_fits_the_shared_wall_angle():
    angle = 0.12
    wall = [(distance * cos(angle), distance * sin(angle)) for distance in (-1.0, -0.5, 0.5, 1.0)]
    assert _wall_yaw_delta((wall, wall), 0.0) == pytest.approx(angle)
