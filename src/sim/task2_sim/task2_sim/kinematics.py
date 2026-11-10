"""ROS-free planar kinematics used by the Task 2 simulator."""

import math


def integrate_pose(x, y, yaw, linear_x, linear_y, angular_z, dt):
    """Advance a planar body-frame velocity by one time step."""
    c, s = math.cos(yaw), math.sin(yaw)
    return (
        x + (c * linear_x - s * linear_y) * dt,
        y + (s * linear_x + c * linear_y) * dt,
        yaw + angular_z * dt,
    )
