import math
from typing import List, Tuple, Optional

from asv_trajectory_planner.vessel_state import VesselState


class MyPlanningAlgorithm:
    """
    自前の経路生成アルゴリズム本体．

    ここではすべてbase_link基準で扱う．

    入力：
        own_state  : base_link基準の自船状態
        other_state: base_link基準の他船状態
        goal       : base_link基準のゴール位置

    出力：
        points_base = [(x1, y1), (x2, y2), ...]
    """

    def __init__(
        self,
        point_spacing: float = 0.5,
        avoid_radius: float = 2.0,
        avoid_offset: float = 3.0,
    ):
        self.point_spacing = point_spacing
        self.avoid_radius = avoid_radius
        self.avoid_offset = avoid_offset

    def plan(
        self,
        own_state: VesselState,
        other_state: Optional[VesselState],
        goal: Tuple[float, float],
    ) -> List[Tuple[float, float]]:
        start = (own_state.x, own_state.y)

        if other_state is None:
            key_points = [start, goal]
        else:
            key_points = self._build_key_points(
                start=start,
                goal=goal,
                other=(other_state.x, other_state.y),
            )

        points_base = self._interpolate_points(key_points)

        return points_base

    def _build_key_points(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        other: Tuple[float, float],
    ) -> List[Tuple[float, float]]:
        sx, sy = start
        gx, gy = goal
        ox, oy = other

        vx = gx - sx
        vy = gy - sy
        length = math.hypot(vx, vy)

        if length < 1e-6:
            return [start]

        wx = ox - sx
        wy = oy - sy

        t = (wx * vx + wy * vy) / (length * length)

        if t < 0.0 or t > 1.0:
            return [start, goal]

        closest_x = sx + t * vx
        closest_y = sy + t * vy

        distance_to_line = math.hypot(ox - closest_x, oy - closest_y)

        if distance_to_line > self.avoid_radius:
            return [start, goal]

        cross = vx * (oy - sy) - vy * (ox - sx)

        if cross >= 0.0:
            side = -1.0
        else:
            side = 1.0

        perp_x = -vy / length
        perp_y = vx / length

        detour_x = closest_x + side * self.avoid_offset * perp_x
        detour_y = closest_y + side * self.avoid_offset * perp_y

        return [start, (detour_x, detour_y), goal]

    def _interpolate_points(
        self,
        key_points: List[Tuple[float, float]],
    ) -> List[Tuple[float, float]]:
        if len(key_points) <= 1:
            return key_points

        dense_points: List[Tuple[float, float]] = []

        for i in range(len(key_points) - 1):
            x0, y0 = key_points[i]
            x1, y1 = key_points[i + 1]

            dx = x1 - x0
            dy = y1 - y0
            length = math.hypot(dx, dy)

            num_divisions = max(1, math.ceil(length / self.point_spacing))

            if i == 0:
                dense_points.append((x0, y0))

            for j in range(1, num_divisions + 1):
                ratio = j / num_divisions

                x = x0 + ratio * dx
                y = y0 + ratio * dy

                dense_points.append((x, y))

        return dense_points