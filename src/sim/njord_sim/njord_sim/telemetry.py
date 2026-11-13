"""Bounded trajectory sampling independent of ROS transport."""

import math


class TrajectorySampler:
    """Keep turns and movement, without recording stationary sensor jitter."""

    def __init__(self, max_points=600, distance=0.10, angle=0.05):
        self.max_points = max_points
        self.distance = distance
        self.angle = angle
        self.last = None

    def accept(self, x, y, yaw):
        if self.last is not None:
            px, py, pa = self.last
            delta = math.atan2(math.sin(yaw - pa), math.cos(yaw - pa))
            if math.hypot(x - px, y - py) < self.distance and abs(delta) < self.angle:
                return False
        self.last = (x, y, yaw)
        return True
