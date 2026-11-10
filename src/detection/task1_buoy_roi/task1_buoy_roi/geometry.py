import json
import math


def parse_positions(value):
    try:
        positions = json.loads(value)
        return [(float(point[0]), float(point[1])) for point in positions if len(point) >= 2]
    except (TypeError, ValueError, json.JSONDecodeError):
        return []


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def nearest_roi(x, y, yaw, positions, activation_range):
    candidates = [(math.hypot(bx - x, by - y), bx, by) for bx, by in positions]
    if not candidates:
        return None
    distance, bx, by = min(candidates)
    if distance > activation_range:
        return None
    bearing = math.atan2(by - y, bx - x) - yaw
    return distance, math.atan2(math.sin(bearing), math.cos(bearing))
