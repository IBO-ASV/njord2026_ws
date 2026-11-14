from dataclasses import dataclass, replace
from math import pi

import pytest

from waypoint_transformer import transform_waypoints


@dataclass(frozen=True)
class Point:
    x: float
    y: float
    yaw: float


def test_transform_anchors_rotates_and_scales_waypoints():
    transformed = transform_waypoints(
        (Point(10.0, 5.0, 0.0), Point(12.0, 5.0, 0.5)), 3.0, 4.0, pi / 2, 0.5, replace
    )
    assert transformed[0] == Point(3.0, 4.0, pi / 2)
    assert transformed[1].x == 3.0
    assert transformed[1].y == 5.0
    assert transformed[1].yaw == 0.5 + pi / 2


def test_transform_rejects_negative_scale():
    with pytest.raises(ValueError, match="positive"):
        transform_waypoints((Point(0.0, 0.0, 0.0),), 0.0, 0.0, 0.0, -0.1, replace)
