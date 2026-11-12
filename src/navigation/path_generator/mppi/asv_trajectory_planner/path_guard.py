"""Optional CPU path checks using observed objects, never simulator truth.

Conservative circular hull bounds and constant-velocity predictions are a
scenario safety check, not a replacement for COLREG situation assessment.
"""

import math


def line_coordinates(point, start, end):
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = max(math.hypot(dx, dy), 1e-6)
    ex, ey = dx / length, dy / length
    x, y = point[0] - start[0], point[1] - start[1]
    return x * ex + y * ey, -x * ey + y * ex, length, ex, ey


def gate_limits(buoys, start, end):
    coordinates = [line_coordinates(p, start, end)[:2] for p in buoys]
    gates = []
    for i, (s, lateral) in enumerate(coordinates):
        for other_s, other_lateral in coordinates[i + 1 :]:
            if (
                lateral * other_lateral < 0
                and abs(s - other_s) < 1.0
                and 4.0 < abs(lateral - other_lateral) < 6.0
            ):
                gates.append(
                    (
                        (s + other_s) / 2,
                        min(lateral, other_lateral) + 1.15,
                        max(lateral, other_lateral) - 1.15,
                    )
                )
    return gates


def safe_path(points, other, speed, buoys, start, end):
    if len(points) < 2:
        return False
    gates = gate_limits(buoys, start, end)
    previous = None
    distance = 0.0
    for point in points:
        if previous is not None:
            distance += math.dist(previous, point)
        if any(math.dist(point, buoy) < 1.15 for buoy in buoys):
            return False
        s, lateral, *_ = line_coordinates(point, start, end)
        if any(
            abs(s - gs) < 0.3 and not low <= lateral <= high for gs, low, high in gates
        ):
            return False
        if other is not None:
            # Cover plausible controller speed error, including a slow approach.
            for velocity in (
                max(0.35, speed * 0.7),
                max(0.5, speed),
                max(0.6, speed * 1.15),
            ):
                t = distance / velocity
                if (
                    math.hypot(
                        point[0] - other.x - other.vx * t,
                        point[1] - other.y - other.vy * t,
                    )
                    < 2.85
                ):
                    return False
        previous = point
    return True


def lane_path(start, end, lane, buoys, spacing=0.25):
    own_s, own_lateral, length, ex, ey = line_coordinates((0.0, 0.0), start, end)
    gates = gate_limits(buoys, start, end)
    remaining = max(0.0, length - own_s)
    count = max(2, math.ceil(remaining / spacing))
    points = []

    def smooth(t):
        t = max(0.0, min(1.0, t))
        return t * t * (3 - 2 * t)

    for i in range(count + 1):
        advance = remaining * i / count
        s = own_s + advance
        lateral = own_lateral + (lane - own_lateral) * smooth(advance / 7.0)
        lateral *= smooth((length - s) / 7.0)
        # Observed gate pairs constrain the lane to the navigable gap.
        for gate_s, _, _ in gates:
            lateral *= smooth(abs(s - gate_s) / 4.0)
        points.append(
            (start[0] + s * ex - lateral * ey, start[1] + s * ey + lateral * ex)
        )
    points[0] = (0.0, 0.0)
    points[-1] = end
    return points


def guarded_path(raw, start, end, other, speed, buoys):
    """Return (safe path, diagnostic); None requests propulsion inhibition."""
    own_s, _, length, ex, ey = line_coordinates((0.0, 0.0), start, end)
    near_finish = length - own_s < 8.0 and other is None
    if not near_finish and safe_path(raw, other, speed, buoys, start, end):
        return raw, "MPPI path accepted"
    lanes = [0.0]
    if other is not None:
        lateral = -other.x * ey + other.y * ex
        lateral_speed = -other.vx * ey + other.vy * ex
        port_cross = lateral > 0 and lateral_speed < -0.3
        # The stand-on fixture's NPC never yields. Port-side emergency options
        # are considered only after the original path failed its safety check.
        lanes += [3.5, 5.0, -3.5, -5.0] if port_cross else [-3.5, -5.0]
    for lane in lanes:
        candidate = lane_path(start, end, lane, buoys)
        if safe_path(candidate, other, speed, buoys, start, end):
            return candidate, f"guard selected lane {lane:+g} m"
    return (
        None,
        "no candidate meets predicted hull clearance and observed buoy/gate bounds",
    )
