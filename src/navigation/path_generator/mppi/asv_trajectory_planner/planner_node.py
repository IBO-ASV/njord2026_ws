#!/usr/bin/env python3

import math
import time

import numpy as np
import rclpy
import torch
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import Bool

from tf2_ros import Buffer, TransformListener

try:
    from tf2_ros import TransformException
except ImportError:
    from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
    TransformException = (LookupException, ConnectivityException, ExtrapolationException)

from asv_trajectory_planner.trajectory_generator import TrajectoryGenerator
from asv_trajectory_planner import crm_torch


class PlannerNode(Node):
    """
    自船Odometry，他船TF，他船TwistStamped，2つのウェイポイントを受け取り，
    自船が追従すべきPathをpublishするノード．

    Subscribe:
        /own_ship/odom
        /other_ship/twist
        /waypoint1_pose
        /waypoint2_pose

    TF:
        base_link -> other_ship_base_link

    Publish:
        /planned_path
    """

    def __init__(self):
        super().__init__("planner_node")

        # Topics
        self.declare_parameter("own_odom_topic", "/own_ship/odom")
        self.declare_parameter("other_ship_twist_topic", "/other_ship/twist")
        self.declare_parameter("opponent_detection_topic", "/task2/opponent_detected")
        self.declare_parameter("require_opponent_detection_status", False)
        self.declare_parameter("opponent_input_timeout_sec", 1.5)
        self.declare_parameter("waypoint1_topic", "/waypoint1_pose")
        self.declare_parameter("waypoint2_topic", "/waypoint2_pose")
        self.declare_parameter("path_topic", "/planned_path")
        self.declare_parameter("buoy_detections_topic", "/buoy_detections")
        self.declare_parameter("use_detected_buoys", True)
        self.declare_parameter("use_virtual_buoys", False)
        self.declare_parameter("buoy_detection_timeout_sec", 1.0)
        self.declare_parameter("buoy_merge_distance_m", 1.0)

        # TF frames
        self.declare_parameter("own_frame", "base_link")
        self.declare_parameter("other_ship_frame", "other_ship_base_link")

        # Planner parameters
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("planning_frequency", 2.0)
        self.declare_parameter("point_spacing", 0.5)
        self.declare_parameter("avoid_radius", 2.0)
        self.declare_parameter("avoid_offset", 3.0)
        self.declare_parameter("require_other_ship", True)
        self.declare_parameter("other_twist_is_relative", True)
        self.declare_parameter("reconnect_line_distance_m", 1.0)
        self.declare_parameter("reconnect_ahead_length_m", 8.0)
        self.declare_parameter("straight_path_spacing_m", 2.0)
        self.declare_parameter("straight_path_length_m", 60.0)
        self.declare_parameter("mppi_smoothing_window", 5)
        self.declare_parameter("opponent_use_distance_m", 20.0)
        self.declare_parameter("opponent_passed_margin_m", 2.0)

        # ------------------------------------------------------------
        # MPPI hyperparameters.
        # Defaults equal the values that were hardcoded in
        # asv_trajectory_planner/mppi_torch.py, so leaving these untouched
        # keeps the planner behaviorally identical to the previous version.
        # See config/mppi_params.yaml for the parameter -> code mapping.
        # ------------------------------------------------------------
        self.declare_parameter("mppi.horizon", 225)
        self.declare_parameter("mppi.dt", 0.1)
        self.declare_parameter("mppi.num_samples", 5000)
        self.declare_parameter("mppi.lambda", 12.0)
        self.declare_parameter("mppi.control_noise_sigma", [35.0, 0.0])
        self.declare_parameter("mppi.target_speed", 2.0 * 1852.0 / 3600.0)
        self.declare_parameter("mppi.path_cost_weight", 150.0)
        # collision cost is a (min, max) scaling pair in the code
        # (col_cost_min / col_cost_max), not a single weight.
        self.declare_parameter("mppi.collision_cost_min", 10.0)
        self.declare_parameter("mppi.collision_cost_max", 50.0)
        self.declare_parameter("mppi.gate_cost_weight", 3.0)
        self.declare_parameter("mppi.buoy_cost_weight", 120.0)
        self.declare_parameter("mppi.speed_cost_weight", 0.1)
        self.declare_parameter("mppi.control_cost_weight", 0.2)
        # "safe distance" in the code is the CRM bumper ellipse, expressed as
        # per-side gains in multiples of LOA, plus buoy/gate geometry.
        self.declare_parameter("mppi.own_loa_m", 1.2)
        self.declare_parameter("mppi.other_loa_m", 2.0)
        self.declare_parameter("mppi.safe_distance_right_loa", 3.2)
        self.declare_parameter("mppi.safe_distance_left_loa", 1.6)
        self.declare_parameter("mppi.safe_distance_fore_loa", 6.4)
        self.declare_parameter("mppi.safe_distance_aft_loa", 1.6)
        self.declare_parameter("mppi.gate_half_width_m", 4.0)
        self.declare_parameter("mppi.buoy_margin_m", 1.0)
        self.declare_parameter("mppi.buoy_longitudinal_sigma_m", 8.0)
        # Visualization of the CRM values actually evaluated by MPPI.  This
        # is deliberately separate from Nav2's obstacle costmap.
        self.declare_parameter("mppi.crm_costmap_topic", "/mppi/crm_costmap")
        # CRM is a high-level collision-risk field, not an obstacle map.
        # A 0.5 m grid is sufficient and keeps the per-cycle full-map
        # time-domain evaluation lightweight.
        self.declare_parameter("mppi.crm_costmap_resolution_m", 0.5)
        self.declare_parameter("mppi.crm_costmap_width_m", 120.0)
        self.declare_parameter("mppi.crm_costmap_height_m", 60.0)
        self.declare_parameter("mppi.crm_costmap_origin_x_m", -20.0)
        self.declare_parameter("mppi.crm_costmap_origin_y_m", -30.0)

        self.own_odom_topic = self.get_parameter("own_odom_topic").value
        self.other_ship_twist_topic = self.get_parameter("other_ship_twist_topic").value
        self.opponent_detection_topic = self.get_parameter("opponent_detection_topic").value
        self.require_opponent_detection_status = self.get_parameter(
            "require_opponent_detection_status").value
        self.opponent_input_timeout_sec = float(self.get_parameter(
            "opponent_input_timeout_sec").value)
        self.waypoint1_topic = self.get_parameter("waypoint1_topic").value
        self.waypoint2_topic = self.get_parameter("waypoint2_topic").value
        self.path_topic = self.get_parameter("path_topic").value
        self.buoy_detections_topic = self.get_parameter("buoy_detections_topic").value
        self.use_detected_buoys = self.get_parameter("use_detected_buoys").value
        self.use_virtual_buoys = self.get_parameter("use_virtual_buoys").value
        self.buoy_detection_timeout_sec = float(
            self.get_parameter("buoy_detection_timeout_sec").value)
        self.buoy_merge_distance_m = float(
            self.get_parameter("buoy_merge_distance_m").value)

        self.own_frame = self.get_parameter("own_frame").value
        self.other_ship_frame = self.get_parameter("other_ship_frame").value

        self.frame_id = self.get_parameter("frame_id").value
        planning_frequency = self.get_parameter("planning_frequency").value
        point_spacing = self.get_parameter("point_spacing").value
        avoid_radius = self.get_parameter("avoid_radius").value
        avoid_offset = self.get_parameter("avoid_offset").value
        self.require_other_ship = self.get_parameter("require_other_ship").value
        other_twist_is_relative = self.get_parameter("other_twist_is_relative").value
        reconnect_line_distance_m = self.get_parameter("reconnect_line_distance_m").value
        reconnect_ahead_length_m = self.get_parameter("reconnect_ahead_length_m").value
        straight_path_spacing_m = self.get_parameter("straight_path_spacing_m").value
        straight_path_length_m = self.get_parameter("straight_path_length_m").value
        mppi_smoothing_window = self.get_parameter("mppi_smoothing_window").value
        opponent_use_distance_m = self.get_parameter("opponent_use_distance_m").value
        opponent_passed_margin_m = self.get_parameter("opponent_passed_margin_m").value

        # Keyword names match MPPIPlanner's constructor arguments.
        mppi_params = {
            "horizon": int(self.get_parameter("mppi.horizon").value),
            "dt": float(self.get_parameter("mppi.dt").value),
            "num_samples": int(self.get_parameter("mppi.num_samples").value),
            "lambda_": float(self.get_parameter("mppi.lambda").value),
            "control_noise_sigma": [
                float(v) for v in self.get_parameter("mppi.control_noise_sigma").value
            ],
            "target_speed": float(self.get_parameter("mppi.target_speed").value),
            "path_cost_weight": float(self.get_parameter("mppi.path_cost_weight").value),
            "collision_cost_min": float(self.get_parameter("mppi.collision_cost_min").value),
            "collision_cost_max": float(self.get_parameter("mppi.collision_cost_max").value),
            "gate_cost_weight": float(self.get_parameter("mppi.gate_cost_weight").value),
            "buoy_cost_weight": float(self.get_parameter("mppi.buoy_cost_weight").value),
            "speed_cost_weight": float(self.get_parameter("mppi.speed_cost_weight").value),
            "control_cost_weight": float(self.get_parameter("mppi.control_cost_weight").value),
            "own_loa_m": float(self.get_parameter("mppi.own_loa_m").value),
            "other_loa_m": float(self.get_parameter("mppi.other_loa_m").value),
            "safe_distance_right_loa": float(
                self.get_parameter("mppi.safe_distance_right_loa").value
            ),
            "safe_distance_left_loa": float(
                self.get_parameter("mppi.safe_distance_left_loa").value
            ),
            "safe_distance_fore_loa": float(
                self.get_parameter("mppi.safe_distance_fore_loa").value
            ),
            "safe_distance_aft_loa": float(
                self.get_parameter("mppi.safe_distance_aft_loa").value
            ),
            "gate_half_width_m": float(self.get_parameter("mppi.gate_half_width_m").value),
            "buoy_margin_m": float(self.get_parameter("mppi.buoy_margin_m").value),
            "buoy_longitudinal_sigma_m": float(
                self.get_parameter("mppi.buoy_longitudinal_sigma_m").value
            ),
        }

        self.trajectory_generator = TrajectoryGenerator(
            frame_id=self.frame_id,
            point_spacing=point_spacing,
            avoid_radius=avoid_radius,
            avoid_offset=avoid_offset,
            other_twist_is_relative=other_twist_is_relative,
            opponent_use_distance_m=opponent_use_distance_m,
            opponent_passed_margin_m=opponent_passed_margin_m,
            reconnect_line_distance_m=reconnect_line_distance_m,
            reconnect_ahead_length_m=reconnect_ahead_length_m,
            straight_path_spacing_m=straight_path_spacing_m,
            straight_path_length_m=straight_path_length_m,
            mppi_smoothing_window=mppi_smoothing_window,
            mppi_params=mppi_params,
        )

        self.latest_own_odom = None
        self.latest_other_ship_twist = None
        self.last_other_ship_twist_receipt = None
        self.opponent_detected = False
        self.last_opponent_status_receipt = None
        self.latest_waypoint1_pose = None
        self.latest_waypoint2_pose = None
        # (x, y, receipt time in seconds), all expressed in frame_id.
        self.detected_buoys = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.own_odom_sub = self.create_subscription(
            Odometry,
            self.own_odom_topic,
            self.own_odom_callback,
            10,
        )

        self.other_ship_twist_sub = self.create_subscription(
            TwistStamped,
            self.other_ship_twist_topic,
            self.other_ship_twist_callback,
            10,
        )
        self.opponent_detected_sub = self.create_subscription(
            Bool,
            self.opponent_detection_topic,
            self.opponent_detected_callback,
            10,
        )

        self.waypoint1_sub = self.create_subscription(
            PoseStamped,
            self.waypoint1_topic,
            self.waypoint1_callback,
            10,
        )

        self.waypoint2_sub = self.create_subscription(
            PoseStamped,
            self.waypoint2_topic,
            self.waypoint2_callback,
            10,
        )

        self.buoy_detection_sub = None
        if self.use_detected_buoys:
            self.buoy_detection_sub = self.create_subscription(
                PointStamped,
                self.buoy_detections_topic,
                self.buoy_detection_callback,
                10,
            )

        self.path_pub = self.create_publisher(
            Path,
            self.path_topic,
            10,
        )
        self.crm_costmap_pub = self.create_publisher(
            OccupancyGrid,
            str(self.get_parameter("mppi.crm_costmap_topic").value),
            1,
        )

        timer_period = 1.0 / max(planning_frequency, 1e-6)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("planner_node started.")
        self.get_logger().info(f"Subscribe own_odom        : {self.own_odom_topic}")
        self.get_logger().info(f"Subscribe other_ship_twist: {self.other_ship_twist_topic}")
        self.get_logger().info(f"Subscribe waypoint1      : {self.waypoint1_topic}")
        self.get_logger().info(f"Subscribe waypoint2      : {self.waypoint2_topic}")
        if self.use_detected_buoys:
            self.get_logger().info(f"Subscribe buoy detections: {self.buoy_detections_topic}")
        self.get_logger().info(
            f"Buoys detected/virtual  : {self.use_detected_buoys}/"
            f"{self.use_virtual_buoys}")
        self.get_logger().info(f"TF own_frame             : {self.own_frame}")
        self.get_logger().info(f"TF other_ship_frame      : {self.other_ship_frame}")
        self.get_logger().info(f"Publish planned_path     : {self.path_topic}")
        self.get_logger().info(f"Opponent use distance    : {opponent_use_distance_m} m")
        self.get_logger().info(f"Opponent passed margin   : {opponent_passed_margin_m} m")
        self.get_logger().info(f"Reconnect line distance  : {reconnect_line_distance_m} m")
        self.get_logger().info(f"Reconnect ahead length   : {reconnect_ahead_length_m} m")

    def own_odom_callback(self, msg: Odometry):
        self.latest_own_odom = msg

    def other_ship_twist_callback(self, msg: TwistStamped):
        self.latest_other_ship_twist = msg
        self.last_other_ship_twist_receipt = time.monotonic()

    def _other_twist_in_planner_frame(self, msg: TwistStamped):
        """Express an absolute opponent velocity in the planner global frame.

        Task 2 perception publishes the selected and bearing-clipped opponent
        velocity in ``map``.  The real-vessel planner global frame is ``odom``.
        A velocity is a free vector, so only the TF rotation is applied here;
        translation must not affect it.  The trajectory generator can then use
        the odom-frame ego yaw to rotate the vector into ``base_link``.
        """
        if self.get_parameter("other_twist_is_relative").value:
            return msg

        source_frame = msg.header.frame_id or self.frame_id
        if source_frame == self.frame_id:
            return msg

        try:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.frame_id,
                    source_frame,
                    Time.from_msg(msg.header.stamp),
                ).transform
            except TransformException:
                transform = self.tf_buffer.lookup_transform(
                    self.frame_id, source_frame, Time()
                ).transform
        except TransformException as exc:
            self.get_logger().warning(
                f"Cannot transform opponent velocity {source_frame} -> "
                f"{self.frame_id}: {exc}; ignoring opponent this cycle.",
                throttle_duration_sec=2.0,
            )
            return None

        yaw = self._yaw_from_quaternion(transform.rotation)
        c, s = math.cos(yaw), math.sin(yaw)
        out = TwistStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.twist.linear.x = c * msg.twist.linear.x - s * msg.twist.linear.y
        out.twist.linear.y = s * msg.twist.linear.x + c * msg.twist.linear.y
        out.twist.linear.z = msg.twist.linear.z
        # Planar angular velocity is invariant under a yaw-only frame change.
        out.twist.angular = msg.twist.angular
        return out

    def opponent_detected_callback(self, msg: Bool):
        self.opponent_detected = bool(msg.data)
        self.last_opponent_status_receipt = time.monotonic()

    def _opponent_input_is_fresh(self) -> bool:
        if self.latest_other_ship_twist is None or \
                self.last_other_ship_twist_receipt is None:
            return False
        if time.monotonic() - self.last_other_ship_twist_receipt > \
                self.opponent_input_timeout_sec:
            return False
        if not self.require_opponent_detection_status:
            return True
        return self.opponent_detected and \
            self.last_opponent_status_receipt is not None and \
            time.monotonic() - self.last_opponent_status_receipt <= \
            self.opponent_input_timeout_sec

    def waypoint1_callback(self, msg: PoseStamped):
        self.latest_waypoint1_pose = msg

    def waypoint2_callback(self, msg: PoseStamped):
        self.latest_waypoint2_pose = msg

    def buoy_detection_callback(self, msg: PointStamped):
        """Store a map-frame buoy detection, merging repeated cluster outputs."""
        try:
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    self.frame_id, msg.header.frame_id,
                    Time.from_msg(msg.header.stamp))
            except TransformException:
                tf_msg = self.tf_buffer.lookup_transform(
                    self.frame_id, msg.header.frame_id, Time())
        except TransformException as e:
            self.get_logger().warning(
                f"Cannot transform buoy detection to {self.frame_id}: {e}",
                throttle_duration_sec=2.0)
            return

        q = tf_msg.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        t = tf_msg.transform.translation
        x = t.x + c * msg.point.x - s * msg.point.y
        y = t.y + s * msg.point.x + c * msg.point.y
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        for i, (old_x, old_y, _) in enumerate(self.detected_buoys):
            if math.hypot(x - old_x, y - old_y) <= self.buoy_merge_distance_m:
                self.detected_buoys[i] = (x, y, now_sec)
                return
        self.detected_buoys.append((x, y, now_sec))

    def timer_callback(self):
        if self.latest_own_odom is None:
            self.get_logger().debug("Waiting for own_odom...")
            return

        if self.latest_waypoint1_pose is None:
            self.get_logger().debug("Waiting for waypoint1_pose...")
            return

        if self.latest_waypoint2_pose is None:
            self.get_logger().debug("Waiting for waypoint2_pose...")
            return

        other_transform = None

        other_twist = self.latest_other_ship_twist \
            if self._opponent_input_is_fresh() else None
        if other_twist is not None:
            other_twist = self._other_twist_in_planner_frame(other_twist)
        if other_twist is None:
            if self.require_other_ship:
                self.get_logger().debug("Waiting for other_ship_twist...")
                return
        else:
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    self.own_frame,
                    self.other_ship_frame,
                    Time(),
                )
                other_transform = tf_msg.transform

            except TransformException as e:
                if self.require_other_ship:
                    self.get_logger().warn(
                        f"Cannot lookup TF {self.own_frame} -> "
                        f"{self.other_ship_frame}: {e}",
                        throttle_duration_sec=2.0,
                    )
                    return

                other_transform = None

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self.detected_buoys = [
            buoy for buoy in self.detected_buoys
            if now_sec - buoy[2] <= self.buoy_detection_timeout_sec
        ]
        buoy_positions = [(x, y) for x, y, _ in self.detected_buoys]

        path = self.trajectory_generator.generate(
            own_odom=self.latest_own_odom,
            other_transform=other_transform,
            other_twist=other_twist,
            waypoint1_pose=self.latest_waypoint1_pose,
            waypoint2_pose=self.latest_waypoint2_pose,
            detected_buoys_map=buoy_positions if self.use_detected_buoys else [],
            use_virtual_buoys=bool(self.use_virtual_buoys),
        )

        now = self.get_clock().now().to_msg()
        path.header.stamp = now
        path.header.frame_id = self.frame_id

        for pose in path.poses:
            pose.header.stamp = now
            pose.header.frame_id = self.frame_id

        self.path_pub.publish(path)
        own_position = self.latest_own_odom.pose.pose.position
        self.publish_mppi_crm_costmap(
            own_map_x=own_position.x,
            own_map_y=own_position.y,
            own_map_yaw=self._yaw_from_odom(self.latest_own_odom),
        )

        self.get_logger().info(
            f"Published path: {len(path.poses)} poses, "
            f"detected_buoys={len(buoy_positions)}",
            throttle_duration_sec=2.0,
        )

    @staticmethod
    def _yaw_from_odom(odom: Odometry) -> float:
        q = odom.pose.pose.orientation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def publish_mppi_crm_costmap(
        self, own_map_x: float, own_map_y: float, own_map_yaw: float
    ) -> None:
        """Publish the complete instantaneous CRM field used by MPPI.

        Each forward map cell is evaluated at the time the vessel would reach
        it by continuing straight ahead at its current speed.  The opponent is
        propagated to that same time by ``timedomaincrm``, matching MPPI's
        ``predtimes`` semantics without plotting only sampled trajectories.
        """
        try:
            own_tf = self.tf_buffer.lookup_transform(
                self.frame_id, self.own_frame, Time()
            )
            own_map_x = own_tf.transform.translation.x
            own_map_y = own_tf.transform.translation.y
            own_map_yaw = self._yaw_from_quaternion(own_tf.transform.rotation)
        except TransformException as exc:
            # The odometry pose passed by timer_callback is a safe fallback
            # while map->base_link is still being initialized.
            self.get_logger().warning(
                f"Cannot obtain absolute own-ship TF for CRM: {exc}",
                throttle_duration_sec=2.0,
            )

        resolution = float(self.get_parameter("mppi.crm_costmap_resolution_m").value)
        width_m = float(self.get_parameter("mppi.crm_costmap_width_m").value)
        height_m = float(self.get_parameter("mppi.crm_costmap_height_m").value)
        origin_x = float(self.get_parameter("mppi.crm_costmap_origin_x_m").value)
        origin_y = float(self.get_parameter("mppi.crm_costmap_origin_y_m").value)
        cols, rows = round(width_m / resolution), round(height_m / resolution)

        xs = origin_x + (np.arange(cols) + 0.5) * resolution
        ys = origin_y + (np.arange(rows) + 0.5) * resolution
        map_x, map_y = np.meshgrid(xs, ys)
        own_heading_crm = (-math.degrees(own_map_yaw)) % 360.0
        # Build CRM state directly in absolute map coordinates.  Do not reuse
        # the planner's base_link-relative state: that loses the true map
        # velocity and was the source of the visual offset.
        own_crm = [
            -own_map_y,
            own_map_x,
            # Match the MPPI candidate-state assumption: Task 2 evaluates
            # collision risk at its fixed configured cruise speed (2 kn).
            self.trajectory_generator.planner.target_speed,
            own_heading_crm,
            self.trajectory_generator.planner.own_loa_m,
        ]
        others_crm = self._other_ship_crm_state_in_map()

        crm_x, crm_y = -map_y, map_x
        # Arrival time for a straight, constant-speed continuation of the
        # current course.  A cross-track cell is evaluated at the same arrival
        # time as the forward point beside it; cells behind the vessel are
        # excluded.
        dx, dy = map_x - own_map_x, map_y - own_map_y
        forward_distance = (
            math.cos(own_map_yaw) * dx + math.sin(own_map_yaw) * dy
        )
        own_speed = own_crm[2]
        horizon = (
            self.trajectory_generator.planner.mppi_controller.horizon
            * self.trajectory_generator.planner.mppi_controller.pred_dt
        )
        valid_straight = (
            (forward_distance >= 0.0)
            & (forward_distance <= own_speed * horizon)
        )
        if own_speed <= 1.0e-3:
            arrival_time = np.zeros((rows, cols), dtype=np.float32)
            valid_straight[:, :] = False
        else:
            arrival_time = (forward_distance / own_speed).astype(np.float32)
        with torch.no_grad():
            risk = crm_torch.timedomaincrm(
                torch.as_tensor(crm_x, dtype=torch.float32),
                torch.as_tensor(crm_y, dtype=torch.float32),
                torch.as_tensor(arrival_time, dtype=torch.float32),
                own_crm,
                others_crm,
                turn=torch.full(
                    (rows, cols), math.radians(own_heading_crm),
                    dtype=torch.float32,
                ),
                ax_gains=self.trajectory_generator.planner.ax_gains,
            ).cpu().numpy()
        risk[~valid_straight] = 0.0

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.info.resolution = resolution
        msg.info.width = cols
        msg.info.height = rows
        msg.info.origin.position.x = origin_x
        msg.info.origin.position.y = origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = np.rint(np.clip(risk, 0.0, 1.0) * 100.0).astype(np.int8).ravel().tolist()
        self.crm_costmap_pub.publish(msg)

    def _other_ship_crm_state_in_map(self):
        """Return opponent CRM state from absolute TF and velocity data."""
        if not self._opponent_input_is_fresh():
            return []
        try:
            other_tf = self.tf_buffer.lookup_transform(
                self.frame_id, self.other_ship_frame, Time()
            )
            velocity_frame = self.latest_other_ship_twist.header.frame_id or self.frame_id
            velocity_tf = self.tf_buffer.lookup_transform(
                self.frame_id, velocity_frame, Time()
            )
        except TransformException as exc:
            self.get_logger().warning(
                f"Cannot build absolute CRM state: {exc}",
                throttle_duration_sec=2.0,
            )
            return []

        position = other_tf.transform.translation
        other_yaw = self._yaw_from_quaternion(other_tf.transform.rotation)
        velocity_yaw = self._yaw_from_quaternion(velocity_tf.transform.rotation)
        twist = self.latest_other_ship_twist.twist
        # Convert the reported velocity into map coordinates before projecting
        # it onto the opponent's map-frame heading (VesselState.u).
        vx = math.cos(velocity_yaw) * twist.linear.x - math.sin(velocity_yaw) * twist.linear.y
        vy = math.sin(velocity_yaw) * twist.linear.x + math.cos(velocity_yaw) * twist.linear.y
        other_u = math.cos(other_yaw) * vx + math.sin(other_yaw) * vy
        return [[
            -position.y,
            position.x,
            other_u,
            (-math.degrees(other_yaw)) % 360.0,
            self.trajectory_generator.planner.other_loa_m,
        ]]

    @staticmethod
    def _yaw_from_quaternion(q) -> float:
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )


def main(args=None):
    rclpy.init(args=args)

    node = PlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
