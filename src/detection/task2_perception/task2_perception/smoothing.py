"""Exponential low-pass + spike rejection for the opponent twist output.

Ports the filtering style of
src/navigation/path_generator/mppi/asv_trajectory_planner/opponent_twist_from_tf_node.py
(lowpass_alpha blending + max-speed spike gate) into a reusable, ROS-free
class. NOTE: the input here is the tracker's EKF velocity — we never
numerically differentiate positions.
"""

from __future__ import annotations

import math


KNOT_TO_MPS = 1852.0 / 3600.0


def knots_to_mps(speed_knots: float) -> float:
    """Convert knots to the SI velocity unit used by geometry_msgs/Twist."""
    return float(speed_knots) * KNOT_TO_MPS


def mps_to_knots(speed_mps: float) -> float:
    """Convert an SI velocity to knots for operator-facing values."""
    return float(speed_mps) / KNOT_TO_MPS


class TwistSmoother:
    """First-order IIR low-pass with spike rejection for planar twists.

    update() returns the filtered (vx, vy, wz) or None when the sample was
    rejected as a spike (the caller should then publish nothing / keep silent,
    mirroring opponent_twist_from_tf_node behavior).
    """

    def __init__(self, alpha: float = 0.3, max_speed_mps: float = 5.0,
                 max_yaw_rate_rps: float = 1.5):
        self.alpha = min(max(float(alpha), 0.0), 1.0)
        self.max_speed_mps = float(max_speed_mps)
        self.max_yaw_rate_rps = float(max_yaw_rate_rps)
        self._vx = None
        self._vy = None
        self._wz = None

    def reset(self):
        """Forget the filter state (call when the selected track changes)."""
        self._vx = None
        self._vy = None
        self._wz = None

    def update(self, vx: float, vy: float, wz: float = 0.0):
        """Feed one raw sample; returns (vx, vy, wz) filtered, or None on spike."""
        speed = math.hypot(vx, vy)
        if speed > self.max_speed_mps or abs(wz) > self.max_yaw_rate_rps:
            # Spike: ignore the sample entirely (do not pollute the filter).
            return None
        if self._vx is None:
            self._vx, self._vy, self._wz = float(vx), float(vy), float(wz)
        else:
            a = self.alpha
            self._vx = a * vx + (1.0 - a) * self._vx
            self._vy = a * vy + (1.0 - a) * self._vy
            self._wz = a * wz + (1.0 - a) * self._wz
        return self._vx, self._vy, self._wz
