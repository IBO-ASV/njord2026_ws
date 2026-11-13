"""Occlusion-bounded free-space rays for the sim's physical costmap sensor.

This supplements the existing 100 m map-frame obstacle stand-in. It does not
feed the 40 m recognition LiDAR or the MPPI perception pipeline.
"""

import math

import numpy as np

SCAN_BEAMS = 8192
SCAN_RANGE = 100.0
SCAN_MIN_RANGE = 0.1
SCAN_ANGLE_MIN = -math.pi
SCAN_ANGLE_INCREMENT = math.tau / SCAN_BEAMS


def clearing_ranges(pose, buoys, opponent=None):
    """Nearest buoy/rectangle intersection; no returns beyond physical surfaces.

    pose is the costmap sensor's published (x, y, yaw). Buoys are (x,y,...)
    with the fixture's 0.175 m physical radius. Opponent is (x,y,yaw,length,width).
    A small margin keeps clearing endpoints in front of actual obstacle cells.
    """
    x, y, yaw = pose
    angles = SCAN_ANGLE_MIN + np.arange(SCAN_BEAMS) * SCAN_ANGLE_INCREMENT + yaw
    dx, dy = np.cos(angles), np.sin(angles)
    # Laser projection drops values equal to range_max; use finite no-hit rays
    # just inside its limit, rather than treating unknown space as an obstacle.
    nearest = np.full(SCAN_BEAMS, SCAN_RANGE - 0.001)
    for buoy in buoys:
        rx, ry = buoy[0] - x, buoy[1] - y
        radius2 = 0.175**2
        if rx * rx + ry * ry <= radius2:
            nearest[:] = 0.0
            break
        projection = rx * dx + ry * dy
        discriminant = radius2 - (rx * rx + ry * ry - projection**2)
        distance = projection - np.sqrt(np.maximum(discriminant, 0.0))
        hit = (discriminant >= 0.0) & (distance >= 0.0)
        nearest = np.minimum(
            nearest, np.where(hit, np.maximum(0.0, distance - 0.03), np.inf)
        )
    if opponent is not None:
        ox, oy, heading, length, width = opponent
        c, s = math.cos(heading), math.sin(heading)
        origins = ((x - ox) * c + (y - oy) * s, -(x - ox) * s + (y - oy) * c)
        directions = (dx * c + dy * s, -dx * s + dy * c)
        near, far = np.zeros(SCAN_BEAMS), np.full(SCAN_BEAMS, np.inf)
        for origin, direction, half in zip(
            origins, directions, (length / 2, width / 2)
        ):
            parallel = np.abs(direction) < 1e-12
            first = np.divide(
                -half - origin,
                direction,
                out=np.full(SCAN_BEAMS, -np.inf),
                where=~parallel,
            )
            last = np.divide(
                half - origin,
                direction,
                out=np.full(SCAN_BEAMS, np.inf),
                where=~parallel,
            )
            axis_near, axis_far = np.minimum(first, last), np.maximum(first, last)
            if abs(origin) > half:
                axis_near[parallel], axis_far[parallel] = np.inf, -np.inf
            near, far = np.maximum(near, axis_near), np.minimum(far, axis_far)
        hit = far >= near
        nearest = np.minimum(
            nearest, np.where(hit, np.maximum(0.0, near - 0.03), np.inf)
        )
    nearest[nearest < SCAN_MIN_RANGE] = np.nan
    return nearest.astype(np.float32)
