"""Deterministic Task 3 lateral-mark virtual-wall geometry."""

import math


def lateral_wall_points(color, x, y, heading, radius=2.0, points_per_full_circle=40):
    """Return the outer half-circle for one Region-A red or green lateral mark."""
    if color not in ("red", "green") or not math.isfinite(heading) or radius < 0.0:
        return []
    # Red is port/left; green is starboard/right.  Keep the channel-facing
    # hemisphere free and fill only the outer side.
    start = heading if color == "red" else heading - math.pi
    count = round(points_per_full_circle / 2.0) + 1
    return [
        (x + radius * math.cos(start + math.pi * index / (count - 1)),
         y + radius * math.sin(start + math.pi * index / (count - 1)), 0.0)
        for index in range(count)
    ]


def same_color_wall_points(points, heading, max_gap_m=12.0, point_spacing_m=0.2):
    """Connect nearby same-colour marks in channel order without bridging gaps."""
    if not math.isfinite(heading) or max_gap_m <= 0.0 or point_spacing_m <= 0.0:
        return []
    along = (math.cos(heading), math.sin(heading))
    ordered = sorted(points, key=lambda point: point[0] * along[0] + point[1] * along[1])
    wall = []
    for first, second in zip(ordered, ordered[1:]):
        dx, dy = second[0] - first[0], second[1] - first[1]
        distance = math.hypot(dx, dy)
        if distance <= 0.0 or distance > max_gap_m:
            continue
        intervals = max(1, math.ceil(distance / point_spacing_m))
        wall.extend(
            (first[0] + dx * index / intervals, first[1] + dy * index / intervals, 0.0)
            for index in range(intervals + 1)
        )
    return wall


def task3_virtual_wall_points(buoy_positions, radius=2.0, max_gap_m=12.0, point_spacing_m=0.2):
    """Build walls for both point-symmetric Task 3 courses in the map frame."""
    groups = {
        ("b31", "red"): [], ("b31", "green"): [],
        ("b32", "red"): [], ("b32", "green"): [],
    }
    for buoy in buoy_positions:
        name = buoy["name"]
        course = "b31" if name.startswith("b31_") else "b32" if name.startswith("b32_") else None
        color = "red" if "_red_" in name else "green" if "_green_" in name else None
        if course and color:
            groups[(course, color)].append((buoy["x"], buoy["y"], 0.0))

    wall = []
    for course, heading in (("b31", 0.0), ("b32", math.pi)):
        for color in ("red", "green"):
            marks = groups[(course, color)]
            for x, y, _ in marks:
                wall.extend(lateral_wall_points(color, x, y, heading, radius))
            wall.extend(same_color_wall_points(marks, heading, max_gap_m, point_spacing_m))
    return wall
