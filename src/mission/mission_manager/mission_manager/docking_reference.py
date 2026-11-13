"""Geometric SE(2) reference; orientations describe the bow, not the tangent."""
from dataclasses import replace
from math import atan2, ceil, cos, hypot, isfinite, sin


def docking_reference(start, goal, spacing=0.05):
    if spacing <= 0 or not all(isfinite(v) for v in (
        start.x, start.y, start.yaw, goal.x, goal.y, goal.yaw, spacing
    )):
        raise ValueError("docking reference requires finite poses and positive spacing")
    delta = atan2(sin(goal.yaw - start.yaw), cos(goal.yaw - start.yaw))
    count = max(2, ceil(hypot(goal.x-start.x, goal.y-start.y)/spacing),
                ceil(abs(delta)/0.02))
    return tuple(replace(
        goal, x=start.x + t*(goal.x-start.x), y=start.y + t*(goal.y-start.y),
        yaw=atan2(sin(start.yaw + (3*t*t-2*t*t*t)*delta),
                  cos(start.yaw + (3*t*t-2*t*t*t)*delta)),
    ) for t in (i/count for i in range(count+1)))
