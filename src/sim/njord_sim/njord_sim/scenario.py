"""Survey-derived geometry shared by simulation, mission projection and scoring."""

from dataclasses import dataclass, replace
from math import atan2, cos, sin, hypot
import json
import yaml
from pathlib import Path
from mission_manager.geodesy import HomeDatum, wgs84_to_enu
from mission_manager.task_registry import TaskRegistry
from mission_manager.waypoint_config import WaypointConfigLoader, Waypoint, Route

TASKS = ("task1", "task1_2", "task2", "task3_1", "task3_2", "task4")


@dataclass
class Scenario:
    task: object
    route: object
    datum: object
    buoys: list
    docks: list
    initial: tuple


def scenario(task_id):
    from ament_index_python.packages import get_package_share_directory

    registry = TaskRegistry.from_file(
        Path(get_package_share_directory("mission_manager"))
        / "config/task_registry.yaml"
    )
    task = registry.get(task_id)
    if task_id not in TASKS:
        raise ValueError(f"unsupported scenario: {task_id}")
    loader = WaypointConfigLoader()
    path = Path(get_package_share_directory(task.route_package)) / task.route
    original = loader.load(path, task.route_key)
    first = original.waypoints[0]
    datum = HomeDatum(first.latitude, first.longitude, 0.0)
    route = WaypointConfigLoader(datum).load(path, task.route_key)
    if task_id == "task1_2":
        start = next(
            i for i, w in enumerate(route.waypoints) if w.competition_id == "3"
        )
        route = replace(route, waypoints=route.waypoints[start:])
    buoys = []
    if task_id.startswith("task1"):
        # The live YAML currently holds a short Japanese field-test route.
        # Competition simulation retains the surveyed course already in task1_sim.
        params = yaml.safe_load(
            (
                Path(get_package_share_directory("task1_sim"))
                / "config/task1_params.yaml"
            ).read_text()
        )["task1_orchestrator"]["ros__parameters"]
        gps = json.loads(params["gps_checkpoint_xy"])
        mid = json.loads(params["waypoint1_xy"])
        cardinal = json.loads(params["waypoint2_xy"])
        positions = [gps[0], *mid, gps[1], gps[2], *cardinal, gps[3]]
        ids = [
            "1",
            *[f"1.{i + 1}" for i in range(len(mid))],
            "2",
            "3",
            *[f"3.{i + 1}" for i in range(len(cardinal))],
            "4",
        ]
        datum = HomeDatum(63.4408000000, 10.4233194444, 0.0)
        waypoints = []
        for i, ((x, y), checkpoint) in enumerate(zip(positions, ids)):
            lat, lon = inverse_enu(x, y, datum)
            target = positions[min(i + 1, len(positions) - 1)]
            angle = (
                atan2(target[1] - y, target[0] - x)
                if i + 1 < len(positions)
                else waypoints[-1].yaw
            )
            waypoints.append(
                Waypoint(
                    checkpoint,
                    x,
                    y,
                    angle,
                    "GPS " + checkpoint,
                    "goal" if checkpoint == "4" else "waypoint",
                    competition_id=checkpoint,
                    latitude=lat,
                    longitude=lon,
                )
            )
        if task_id == "task1_2":
            waypoints = waypoints[ids.index("3") :]
        route = Route(
            "odom", tuple(waypoints), {}, {}, {"cardinal_marker_detection": True}
        )
        buoys = [
            (*position, mark)
            for position, mark in zip(
                json.loads(params["buoy_position_xy"]), json.loads(params["buoy_marks"])
            )
        ]
    first = route.waypoints[0]
    second = route.waypoints[1]
    yaw = atan2(second.y - first.y, second.x - first.x)
    if task_id in ("task2", "task4"):
        a = (
            route.waypoints[0]
            if task_id == "task2"
            else next(w for w in route.waypoints if w.waypoint_id == "15")
        )
        b = (
            route.waypoints[-1]
            if task_id == "task2"
            else next(w for w in route.waypoints if w.waypoint_id == "16")
        )
        angle = atan2(b.y - a.y, b.x - a.x)
        offsets = (0.15, 0.85) if task_id == "task2" else (0.5,)
        separation = 2.5 if task_id == "task2" else 3.5
        for fraction in offsets:
            cx, cy = a.x + fraction * (b.x - a.x), a.y + fraction * (b.y - a.y)
            for sign, mark in ((1, "RED"), (-1, "GREEN")):
                buoys.append(
                    (
                        cx - sign * separation * sin(angle),
                        cy + sign * separation * cos(angle),
                        mark,
                    )
                )
    docks = []
    for w in route.waypoints:
        if w.waypoint_type == "dock":
            # Dock mouth is at the previous approach. Axis points into the berth.
            index = route.waypoints.index(w)
            a = route.waypoints[index - 1]
            axis = atan2(w.y - a.y, w.x - a.x)
            docks.append(
                (w.x, w.y, axis, 4.0 if "berth2" in w.waypoint_id else 2.0, 2.0)
            )
    return Scenario(task, route, datum, buoys, docks, (first.x, first.y, yaw))


def dock_segments(dock):
    """Open-mouth rectangular berth, never a closed obstacle over its goal."""
    x, y, yaw, length, width = dock

    def world(u, v):
        return (x + u * cos(yaw) - v * sin(yaw), y + u * sin(yaw) + v * cos(yaw))

    return [
        (world(-length / 2, -width / 2), world(length / 2, -width / 2)),
        (world(length / 2, -width / 2), world(length / 2, width / 2)),
        (world(length / 2, width / 2), world(-length / 2, width / 2)),
    ]


def segment_distance(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    t = max(
        0.0,
        min(
            1.0,
            ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / max(dx * dx + dy * dy, 1e-12),
        ),
    )
    return hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def inverse_enu(x, y, datum):
    """Invert the existing ECEF projection numerically, including ellipsoid scale."""
    lat, lon = datum.latitude, datum.longitude
    for _ in range(5):
        ex, ny = wgs84_to_enu(lat, lon, datum)
        dx = wgs84_to_enu(lat, lon + 1e-6, datum)[0] - ex
        dy = wgs84_to_enu(lat + 1e-6, lon, datum)[1] - ny
        lon += (x - ex) / dx * 1e-6
        lat += (y - ny) / dy * 1e-6
    return lat, lon


def rectangle(x, y, yaw, length=1.5, width=1.0):
    return [
        (x + u * cos(yaw) - v * sin(yaw), y + u * sin(yaw) + v * cos(yaw))
        for u, v in (
            (length / 2, width / 2),
            (length / 2, -width / 2),
            (-length / 2, -width / 2),
            (-length / 2, width / 2),
        )
    ]


def segment_hits_polygon(a, b, polygon):
    """Separating-axis test for a segment against a convex hull footprint."""
    axes = [(-(b[1] - a[1]), b[0] - a[0])]
    axes += [
        (-(q[1] - p[1]), q[0] - p[0])
        for p, q in zip(polygon, polygon[1:] + polygon[:1])
    ]
    for ax, ay in axes:
        if ax * ax + ay * ay < 1e-12:
            continue
        hull = [x * ax + y * ay for x, y in polygon]
        edge = [x * ax + y * ay for x, y in (a, b)]
        if max(hull) < min(edge) or max(edge) < min(hull):
            return False
    return True


def point_rectangle_clearance(px, py, x, y, yaw, radius=0.175):
    dx, dy = px - x, py - y
    u = abs(dx * cos(yaw) + dy * sin(yaw)) - 0.75
    v = abs(-dx * sin(yaw) + dy * cos(yaw)) - 0.5
    return hypot(max(u, 0.0), max(v, 0.0)) + min(max(u, v), 0.0) - radius
