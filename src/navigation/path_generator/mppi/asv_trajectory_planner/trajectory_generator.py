#!/usr/bin/env python3

import math
from typing import List, Optional, Tuple

from geometry_msgs.msg import PoseStamped, Transform, TwistStamped
from nav_msgs.msg import Odometry, Path

from asv_trajectory_planner.mppi_torch import MPPIPlanner
from asv_trajectory_planner.vessel_state import VesselState


class TrajectoryGenerator:
    """
    Task2用 trajectory generator.

    モード:
        1. avoidance:
            他船が20m以内かつ通過前ならMPPI避航Pathを使う。

        2. reconnect:
            他船なし/遠い/通過後で、GPS直線からまだ離れている場合、
            現在位置からGPS直線へ滑らかに戻る短い接続Pathを作る。

        3. straight:
            GPS直線に十分近づいたら、GPS point 1 -> 2 の完全直線Pathを作る。
    """

    def __init__(
        self,
        frame_id: str = "map",
        point_spacing: float = 0.5,
        avoid_radius: float = 2.0,
        avoid_offset: float = 3.0,
        other_twist_is_relative: bool = True,
        opponent_use_distance_m: float = 20.0,
        opponent_passed_margin_m: float = 2.0,
        reconnect_line_distance_m: float = 1.0,
        reconnect_ahead_length_m: float = 8.0,
        straight_path_spacing_m: float = 2.0,
        straight_path_length_m: float = 60.0,
        mppi_smoothing_window: int = 5,
        mppi_params: Optional[dict] = None,
    ):
        self.frame_id = frame_id
        self.other_twist_is_relative = other_twist_is_relative

        self.opponent_use_distance_m = float(opponent_use_distance_m)
        self.opponent_passed_margin_m = float(opponent_passed_margin_m)

        # GPS直線に何m以内まで近づいたら完全直線Pathへ切り替えるか
        self.reconnect_line_distance_m = float(reconnect_line_distance_m)

        # 復帰PathでGPS直線上の何m先を接続目標にするか
        self.reconnect_ahead_length_m = float(reconnect_ahead_length_m)

        self.straight_path_spacing_m = float(straight_path_spacing_m)
        self.straight_path_length_m = float(straight_path_length_m)
        self.mppi_smoothing_window = int(mppi_smoothing_window)

        # mppi_params: optional keyword overrides forwarded verbatim to
        # MPPIPlanner (e.g. horizon, dt, num_samples, cost weights...).
        # None / {} keeps every MPPIPlanner default, i.e. the historical
        # hardcoded values.
        self.planner = MPPIPlanner(
            point_spacing=point_spacing,
            avoid_radius=avoid_radius,
            avoid_offset=avoid_offset,
            **(mppi_params or {}),
        )

    def generate(
        self,
        own_odom: Odometry,
        other_transform: Optional[Transform],
        other_twist: Optional[TwistStamped],
        waypoint1_pose: PoseStamped,
        waypoint2_pose: PoseStamped,
        detected_buoys_map: Optional[List[Tuple[float, float]]] = None,
        use_virtual_buoys: bool = False,
    ) -> Path:
        """
        planningはbase_link基準で行い、最後にmap基準Pathへ変換する。
        """

        # --------------------------------------------------
        # 1. 自船のmap基準姿勢を取得
        # --------------------------------------------------
        own_map_x, own_map_y, own_map_yaw = self._own_pose_in_map(own_odom)

        # --------------------------------------------------
        # 2. 自船状態をbase_link基準で作る
        # --------------------------------------------------
        own_state_base = self._own_odom_to_base_state(own_odom)

        # --------------------------------------------------
        # 3. waypointをmap基準からbase_link基準へ変換
        # --------------------------------------------------
        waypoint1_base = self._map_to_base_link(
            x_m=waypoint1_pose.pose.position.x,
            y_m=waypoint1_pose.pose.position.y,
            own_x=own_map_x,
            own_y=own_map_y,
            own_yaw=own_map_yaw,
        )

        waypoint2_base = self._map_to_base_link(
            x_m=waypoint2_pose.pose.position.x,
            y_m=waypoint2_pose.pose.position.y,
            own_x=own_map_x,
            own_y=own_map_y,
            own_yaw=own_map_yaw,
        )

        detected_buoys_base = [
            self._map_to_base_link(
                x_m=x, y_m=y, own_x=own_map_x, own_y=own_map_y,
                own_yaw=own_map_yaw)
            for x, y in (detected_buoys_map or [])
        ]

        # --------------------------------------------------
        # 4. 他船をMPPIへ渡すか判定
        # --------------------------------------------------
        other_state_base = None

        if other_transform is not None and other_twist is not None:
            use_opponent = self._should_use_opponent(
                own_map_x=own_map_x,
                own_map_y=own_map_y,
                own_map_yaw=own_map_yaw,
                other_transform=other_transform,
                waypoint1_pose=waypoint1_pose,
                waypoint2_pose=waypoint2_pose,
            )

            if use_opponent:
                other_state_base = self._other_tf_twist_to_base_state(
                    own_state_base=own_state_base,
                    other_transform=other_transform,
                    other_twist=other_twist,
                    own_map_yaw=own_map_yaw,
                )

        # --------------------------------------------------
        # 5. Path生成
        # --------------------------------------------------
        if other_state_base is not None or detected_buoys_base or use_virtual_buoys:
            # 他船またはブイあり: MPPI避航Path
            points_base = self.planner.plan(
                own=own_state_base,
                other=other_state_base,
                waypoint1=waypoint1_base,
                waypoint2=waypoint2_base,
                detected_buoys=detected_buoys_base,
                use_virtual_buoys=use_virtual_buoys,
            )

            points_base = self._smooth_points(
                points=points_base,
                window_size=self.mppi_smoothing_window,
            )

        else:
            # 他船なし/遠い/通過後:
            # GPS直線への距離に応じて reconnect または straight を使う
            line_info = self._gps_line_info(
                waypoint1_base=waypoint1_base,
                waypoint2_base=waypoint2_base,
            )

            if line_info is None:
                points_base = [(0.0, 0.0)]

            elif line_info["line_distance"] > self.reconnect_line_distance_m:
                points_base = self._generate_reconnect_then_straight_path_base(
                    line_info=line_info,
                    spacing_m=self.straight_path_spacing_m,
                    straight_length_m=self.straight_path_length_m,
                    reconnect_ahead_length_m=self.reconnect_ahead_length_m,
                )

                # controllerが近すぎる始点を見て旋回しにくくなるのを避ける
                points_base = self._skip_path_head(
                    points=points_base,
                    skip_distance_m=1.3,
                )

            else:
                points_base = self._generate_straight_gps_path_base(
                    line_info=line_info,
                    spacing_m=self.straight_path_spacing_m,
                    max_length_m=self.straight_path_length_m,
                )

        # --------------------------------------------------
        # 6. base_link基準点列をmap基準Pathへ変換
        # --------------------------------------------------
        return self.base_points_to_map_path(
            points_base=points_base,
            own_x=own_map_x,
            own_y=own_map_y,
            own_yaw=own_map_yaw,
        )

    # ============================================================
    # Path generation helpers
    # ============================================================

    def _gps_line_info(
        self,
        waypoint1_base: Tuple[float, float],
        waypoint2_base: Tuple[float, float],
    ):
        """
        base_link基準でGPS直線情報を作る。

        base_link原点 = 現在の自船位置。
        GPS直線上で自船に最も近い点を projection とする。
        """

        x1, y1 = waypoint1_base
        x2, y2 = waypoint2_base

        dx = x2 - x1
        dy = y2 - y1
        length = math.hypot(dx, dy)

        if length < 1e-6:
            return None

        ex = dx / length
        ey = dy / length

        # GPS直線 p(s)=wp1+s*e に対して、原点(0,0)の射影sを求める
        s_proj = -((x1 * ex) + (y1 * ey))
        s_proj = max(0.0, min(s_proj, length))

        proj_x = x1 + s_proj * ex
        proj_y = y1 + s_proj * ey

        line_distance = math.hypot(proj_x, proj_y)

        return {
            "x1": x1,
            "y1": y1,
            "x2": x2,
            "y2": y2,
            "length": length,
            "ex": ex,
            "ey": ey,
            "s_proj": s_proj,
            "proj_x": proj_x,
            "proj_y": proj_y,
            "line_distance": line_distance,
        }

    def _generate_straight_gps_path_base(
        self,
        line_info,
        spacing_m: float,
        max_length_m: float,
    ) -> List[Tuple[float, float]]:
        """
        GPS直線上だけに点を置く完全直線Path。

        GPS直線に十分近づいた後に使う。
        """

        x1 = line_info["x1"]
        y1 = line_info["y1"]
        length = line_info["length"]
        ex = line_info["ex"]
        ey = line_info["ey"]
        s = line_info["s_proj"]

        s_end = min(length, s + max_length_m)
        spacing_m = max(spacing_m, 0.1)

        points: List[Tuple[float, float]] = []

        while s <= s_end + 1e-6:
            points.append((x1 + s * ex, y1 + s * ey))
            s += spacing_m

        if len(points) == 0:
            points.append((line_info["proj_x"], line_info["proj_y"]))

        return points

    def _generate_reconnect_then_straight_path_base(
        self,
        line_info,
        spacing_m: float,
        straight_length_m: float,
        reconnect_ahead_length_m: float,
    ) -> List[Tuple[float, float]]:
        """
        現在位置からGPS直線へ滑らかに戻り、
        その後はGPS直線上を進むPath。

        P0: 現在位置 = base_link原点
        P3: GPS直線上の少し前方点
        P1: 自船前方
        P2: GPS直線方向に沿うようにP3の手前
        """

        x1 = line_info["x1"]
        y1 = line_info["y1"]
        length = line_info["length"]
        ex = line_info["ex"]
        ey = line_info["ey"]
        s_proj = line_info["s_proj"]

        spacing_m = max(spacing_m, 0.1)

        # GPS直線上の少し前方を接続目標にする
        s_join = min(length, s_proj + max(reconnect_ahead_length_m, spacing_m))
        join_x = x1 + s_join * ex
        join_y = y1 + s_join * ey

        p0 = (0.0, 0.0)
        p3 = (join_x, join_y)

        d = math.hypot(p3[0] - p0[0], p3[1] - p0[1])
        ctrl = max(min(0.35 * d, 6.0), 1.0)

        # base_linkのx正方向を現在船首方向とみなして前方に制御点を置く
        p1 = (ctrl, 0.40 * p3[1])  # 再調整：曲がり始めと速度維持のバランス

        # 終端ではGPS直線方向に接続するようにする
        p2 = (p3[0] - ctrl * ex, p3[1] - ctrl * ey)

        n = max(int(d / spacing_m), 4)

        points: List[Tuple[float, float]] = []

        for i in range(n + 1):
            t = i / n
            omt = 1.0 - t

            x = (
                omt ** 3 * p0[0]
                + 3.0 * omt ** 2 * t * p1[0]
                + 3.0 * omt * t ** 2 * p2[0]
                + t ** 3 * p3[0]
            )

            y = (
                omt ** 3 * p0[1]
                + 3.0 * omt ** 2 * t * p1[1]
                + 3.0 * omt * t ** 2 * p2[1]
                + t ** 3 * p3[1]
            )

            points.append((x, y))

        # 接続点以降は完全にGPS直線上
        s = s_join + spacing_m
        s_end = min(length, s_join + straight_length_m)

        while s <= s_end + 1e-6:
            points.append((x1 + s * ex, y1 + s * ey))
            s += spacing_m

        return points


    def _skip_path_head(
        self,
        points,
        skip_distance_m: float,
    ):
        """
        Pathの先頭からskip_distance_mぶんを削る。
        Pure Pursuitのlookahead点が近すぎる問題を避けるために使う。
        """

        if points is None or len(points) < 3:
            return points

        if skip_distance_m <= 0.0:
            return points

        accum = 0.0

        for i in range(1, len(points)):
            x0, y0 = points[i - 1]
            x1, y1 = points[i]

            ds = math.hypot(x1 - x0, y1 - y0)
            accum += ds

            if accum >= skip_distance_m:
                skipped = points[i:]

                if len(skipped) >= 2:
                    return skipped

                return points

        return points


    def _smooth_points(
        self,
        points: List[Tuple[float, float]],
        window_size: int = 5,
    ) -> List[Tuple[float, float]]:
        """
        MPPI避航Pathを軽く平滑化する。
        端点は保存する。
        """

        if points is None or len(points) < 3:
            return points

        if window_size < 3:
            return points

        if window_size % 2 == 0:
            window_size += 1

        half = window_size // 2
        smoothed: List[Tuple[float, float]] = []

        for i in range(len(points)):
            if i == 0 or i == len(points) - 1:
                smoothed.append(points[i])
                continue

            i0 = max(0, i - half)
            i1 = min(len(points), i + half + 1)

            xs = [p[0] for p in points[i0:i1]]
            ys = [p[1] for p in points[i0:i1]]

            smoothed.append((sum(xs) / len(xs), sum(ys) / len(ys)))

        return smoothed

    # ============================================================
    # Opponent gating
    # ============================================================

    def _should_use_opponent(
        self,
        own_map_x: float,
        own_map_y: float,
        own_map_yaw: float,
        other_transform: Transform,
        waypoint1_pose: PoseStamped,
        waypoint2_pose: PoseStamped,
    ) -> bool:
        """
        Trueなら他船をMPPIへ渡す。
        Falseなら他船なしとして、GPS直線復帰/直線追従へ切り替える。
        """

        other_x_base = other_transform.translation.x
        other_y_base = other_transform.translation.y

        distance = math.hypot(other_x_base, other_y_base)

        if distance > self.opponent_use_distance_m:
            return False

        other_x_map, other_y_map = self._base_link_to_map(
            x_b=other_x_base,
            y_b=other_y_base,
            own_x=own_map_x,
            own_y=own_map_y,
            own_yaw=own_map_yaw,
        )

        wp1_x = waypoint1_pose.pose.position.x
        wp1_y = waypoint1_pose.pose.position.y
        wp2_x = waypoint2_pose.pose.position.x
        wp2_y = waypoint2_pose.pose.position.y

        route_dx = wp2_x - wp1_x
        route_dy = wp2_y - wp1_y
        route_len = math.hypot(route_dx, route_dy)

        if route_len < 1e-6:
            return True

        ex = route_dx / route_len
        ey = route_dy / route_len

        own_s = (own_map_x - wp1_x) * ex + (own_map_y - wp1_y) * ey
        other_s = (other_x_map - wp1_x) * ex + (other_y_map - wp1_y) * ey

        # 自船が航路方向で他船より前に出たら通過後
        passed = own_s > other_s + self.opponent_passed_margin_m

        if passed:
            return False

        return True

    # ============================================================
    # State conversion
    # ============================================================

    def _own_pose_in_map(
        self,
        own_odom: Odometry,
    ) -> Tuple[float, float, float]:
        x = own_odom.pose.pose.position.x
        y = own_odom.pose.pose.position.y

        q = own_odom.pose.pose.orientation
        yaw = self._yaw_from_quaternion(q.x, q.y, q.z, q.w)

        return x, y, yaw

    def _own_odom_to_base_state(
        self,
        own_odom: Odometry,
    ) -> VesselState:
        # Task 2 evaluates every MPPI candidate at the configured cruise
        # speed, not at the instantaneous measured surge speed.  This keeps
        # collision prediction conservative while the real vessel is still
        # accelerating and matches the path passed to FollowPath.
        u = self.planner.target_speed
        v = 0.0
        r = math.degrees(own_odom.twist.twist.angular.z)  # ROS rad/s -> MPPI deg/s

        return VesselState(
            x=0.0,
            y=0.0,
            yaw=0.0,
            u=u,
            v=v,
            r=r,
            vx=u,
            vy=v,
        )

    def _other_tf_twist_to_base_state(
        self,
        own_state_base: VesselState,
        other_transform: Transform,
        other_twist: TwistStamped,
        own_map_yaw: float = 0.0,
    ) -> VesselState:
        other_x = other_transform.translation.x
        other_y = other_transform.translation.y

        q = other_transform.rotation
        other_yaw_rad = self._yaw_from_quaternion(q.x, q.y, q.z, q.w)
        other_yaw_deg = math.degrees(other_yaw_rad)

        twist_msg = other_twist.twist if hasattr(other_twist, "twist") else other_twist

        tw_x = twist_msg.linear.x
        tw_y = twist_msg.linear.y
        tw_r_deg = math.degrees(twist_msg.angular.z)

        if self.other_twist_is_relative:
            # tw_x, tw_y が自船基準の相対速度として与えられる場合
            other_vx = own_state_base.vx + tw_x
            other_vy = own_state_base.vy + tw_y
            other_r_deg = own_state_base.r + tw_r_deg
        else:
            # /other_ship/twist は map基準速度としてpublishされている想定。
            # MPPIはbase_link基準で計算するので、map -> base_link へ回転変換する。
            c_own = math.cos(own_map_yaw)
            s_own = math.sin(own_map_yaw)

            other_vx = c_own * tw_x + s_own * tw_y
            other_vy = -s_own * tw_x + c_own * tw_y
            other_r_deg = tw_r_deg

        cy = math.cos(math.radians(other_yaw_deg))
        sy = math.sin(math.radians(other_yaw_deg))

        other_u = cy * other_vx + sy * other_vy
        other_v = -sy * other_vx + cy * other_vy

        # debug: MPPIへ渡す他船状態を確認
        dist_dbg = math.hypot(other_x, other_y)
        print(
            f"[MPPI other debug] "
            f"pos_base=({other_x:.2f}, {other_y:.2f}) m, "
            f"dist={dist_dbg:.2f} m, "
            f"yaw_base={other_yaw_deg:.1f} deg, "
            f"vel_base=({other_vx:.2f}, {other_vy:.2f}) m/s, "
            f"r={other_r_deg:.2f} deg/s"
        )

        return VesselState(
            x=other_x,
            y=other_y,
            yaw=other_yaw_deg,
            u=other_u,
            v=other_v,
            r=other_r_deg,
            vx=other_vx,
            vy=other_vy,
        )

    # ============================================================
    # Coordinate conversion
    # ============================================================

    def base_points_to_map_path(
        self,
        points_base: List[Tuple[float, float]],
        own_x: float,
        own_y: float,
        own_yaw: float,
    ) -> Path:
        path = Path()
        path.header.frame_id = self.frame_id

        points_map: List[Tuple[float, float]] = []

        for x_b, y_b in points_base:
            x_m, y_m = self._base_link_to_map(
                x_b=x_b,
                y_b=y_b,
                own_x=own_x,
                own_y=own_y,
                own_yaw=own_yaw,
            )
            points_map.append((x_m, y_m))

        for i, (x_m, y_m) in enumerate(points_map):
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id

            pose.pose.position.x = x_m
            pose.pose.position.y = y_m
            pose.pose.position.z = 0.0

            yaw_m = self._calc_yaw(points_map, i)

            pose.pose.orientation.z = math.sin(yaw_m / 2.0)
            pose.pose.orientation.w = math.cos(yaw_m / 2.0)

            path.poses.append(pose)

        return path

    def _map_to_base_link(
        self,
        x_m: float,
        y_m: float,
        own_x: float,
        own_y: float,
        own_yaw: float,
    ) -> Tuple[float, float]:
        dx = x_m - own_x
        dy = y_m - own_y

        c = math.cos(own_yaw)
        s = math.sin(own_yaw)

        x_b = c * dx + s * dy
        y_b = -s * dx + c * dy

        return x_b, y_b

    def _base_link_to_map(
        self,
        x_b: float,
        y_b: float,
        own_x: float,
        own_y: float,
        own_yaw: float,
    ) -> Tuple[float, float]:
        c = math.cos(own_yaw)
        s = math.sin(own_yaw)

        x_m = own_x + c * x_b - s * y_b
        y_m = own_y + s * x_b + c * y_b

        return x_m, y_m

    def _calc_yaw(
        self,
        points: List[Tuple[float, float]],
        index: int,
    ) -> float:
        if len(points) <= 1:
            return 0.0

        if index < len(points) - 1:
            x0, y0 = points[index]
            x1, y1 = points[index + 1]
        else:
            x0, y0 = points[index - 1]
            x1, y1 = points[index]

        return math.atan2(y1 - y0, x1 - x0)

    def _yaw_from_quaternion(
        self,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
    ) -> float:
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)

        return math.atan2(siny_cosp, cosy_cosp)
