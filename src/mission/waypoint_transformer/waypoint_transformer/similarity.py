"""Small, dependency-free 2D similarity transform for cart tests."""

from __future__ import annotations

from math import cos, sin
from typing import Sequence, TypeVar


T = TypeVar("T")


def transform_waypoints(waypoints: Sequence[T], anchor_x: float, anchor_y: float,
                        rotation_rad: float, scale: float, replace) -> tuple[T, ...]:
    """Anchor the first waypoint, then translate, rotate, and uniformly scale."""
    if scale <= 0.0:
        raise ValueError("waypoint scale must be positive")
    if not waypoints:
        return ()
    origin = waypoints[0]
    c, s = cos(rotation_rad), sin(rotation_rad)
    return tuple(replace(
        waypoint,
        x=anchor_x + scale * (c * (waypoint.x - origin.x) - s * (waypoint.y - origin.y)),
        y=anchor_y + scale * (s * (waypoint.x - origin.x) + c * (waypoint.y - origin.y)),
        yaw=waypoint.yaw + rotation_rad,
    ) for waypoint in waypoints)
