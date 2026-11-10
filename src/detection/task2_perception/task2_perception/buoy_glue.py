"""ROS-free geometry and gates for the Task 2 route-side buoy detector."""

from __future__ import annotations

import math

import numpy as np


GREEN = "green"
RED = "red"


def classify_route_side(
    point_xy: np.ndarray,
    start_xy: np.ndarray,
    end_xy: np.ndarray,
    expected_offset_m: float,
    offset_tolerance_m: float,
    start_margin_m: float = 0.0,
    end_margin_m: float = 0.0,
) -> str | None:
    """Classify a point near the GPS5->GPS6 side lines.

    Positive lateral offset is left of travel and therefore red; negative is
    right and therefore green.  A point must lie close to either of the two
    parallel lines, not merely somewhere inside a broad corridor.
    """
    start = np.asarray(start_xy, dtype=float)[:2]
    end = np.asarray(end_xy, dtype=float)[:2]
    point = np.asarray(point_xy, dtype=float)[:2]
    route = end - start
    length = float(np.linalg.norm(route))
    if length <= 1e-6:
        return None
    forward = route / length
    delta = point - start
    longitudinal = float(delta @ forward)
    lateral = float(delta[0] * -forward[1] + delta[1] * forward[0])
    if not -float(start_margin_m) <= longitudinal <= length + float(end_margin_m):
        return None
    # Keep the two route sides disjoint even when a broad tolerance is used.
    # A cluster exactly on the centre line has no defensible colour.
    if (lateral > 0.0 and
            abs(lateral - float(expected_offset_m)) <= float(offset_tolerance_m)):
        return RED
    if (lateral < 0.0 and
            abs(lateral + float(expected_offset_m)) <= float(offset_tolerance_m)):
        return GREEN
    return None


def is_stationary(velocity_map: np.ndarray, max_speed_mps: float) -> bool:
    """Return true when the compensated map-frame horizontal speed is small."""
    velocity = np.asarray(velocity_map, dtype=float)
    return math.hypot(float(velocity[0]), float(velocity[1])) <= float(max_speed_mps)
