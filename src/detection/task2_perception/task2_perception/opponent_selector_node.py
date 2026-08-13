#!/usr/bin/env python3
"""Opponent ship selector for Task 2 (node name: opponent_selector).

Subscribes /tracked_objects (ship_perception_msgs/TrackedObjectArray; pose is
base_link-relative, twist is the RELATIVE velocity expressed in the OBJECT
body frame — the submodule tracker's ego compensation is a no-op stub) plus
the ego odometry, selects one opponent via the configured policy, rotates the
twist into base_link axes, ego-compensates, converts to the map frame,
smooths, and publishes:

  * /other_ship/twist  (geometry_msgs/TwistStamped, frame_id = map,
                        ABSOLUTE ground velocity)
  * TF map -> opponent_vessel

No valid track -> both outputs stay SILENT (throttled warning only). The MPPI
planner runs with require_other_ship = False and plans a straight path when
the opponent is absent, so silence is the safe degradation.

Smoothing ports the low-pass + spike-rejection style of
opponent_twist_from_tf_node.py; the velocity source is the tracker EKF (no
numerical differentiation here).
"""

import math

import numpy as np
import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Point, PoseStamped, TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from visualization_msgs.msg import Marker

try:
    from tf2_ros import TransformException
except ImportError:
    from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
    TransformException = (LookupException, ConnectivityException, ExtrapolationException)

try:
    from ship_perception_msgs.msg import TrackedObjectArray
except ImportError:
    # ship_perception_msgs is provided by the pcl_segmentation submodule;
    # a clear message beats a bare ModuleNotFoundError at launch time.
    TrackedObjectArray = None

from task2_perception import cloud_ops, tracking_glue
from task2_perception.smoothing import TwistSmoother, knots_to_mps, mps_to_knots
from task2_perception.tracking_glue import SelectionParams, Track


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class OpponentSelectorNode(Node):

    def __init__(self):
        super().__init__("opponent_selector")

        self.declare_parameters("", [
            ("tracked_objects_topic", "/tracked_objects"),
            ("ego_odom_topic", "/odometry/filtered/local"),
            ("twist_topic", "/other_ship/twist"),
            ("detection_status_topic", "/task2/opponent_detected"),
            ("selected_marker_topic", "/task2/selected_opponent_marker"),
            ("velocity_marker_topic", "/task2/selected_opponent_velocity_marker"),
            ("map_frame", "map"),
            ("base_frame", "base_link"),
            ("opponent_frame", "opponent_vessel"),
            ("confirmed_only", True),
            ("max_distance_m", 60.0),
            ("selection_policy", "nearest"),
            ("target_track_id", -1),
            ("min_length_m", 0.5),
            ("max_length_m", 30.0),
            ("min_width_m", 0.0),
            ("max_width_m", float("inf")),
            ("min_point_count", 5),
            ("stale_timeout_sec", 2.0),
            ("publish_rate_hz", 10.0),
            ("twist_lowpass_alpha", 0.3),
            # Operator-facing speed gates use knots.  The published Twist
            # remains m/s, as required by geometry_msgs/Twist.
            ("min_absolute_speed_knots", 2.0),
            ("max_absolute_speed_knots", 3.0),
            # `straight_line` trusts only a repeatedly observed, low-variance
            # constant-velocity estimate. `standard` preserves legacy gating.
            ("motion_filter_mode", "standard"),
            ("straight_min_hit_count", 15),
            ("straight_max_velocity_stddev_mps", 0.30),
            ("straight_coast_timeout_sec", 2.0),
            # GPS5->GPS6 map-frame target-recognition rectangle.
            ("corridor_enabled", True),
            ("corridor_start_topic", "/waypoint1_pose"),
            ("corridor_end_topic", "/waypoint2_pose"),
            ("corridor_start_offset_m", 5.0),
            ("corridor_end_margin_m", 5.0),
            ("corridor_half_width_m", 20.0),
            ("corridor_ignore_left_side", True),
            ("corridor_left_side_margin_m", 0.0),
            # Restrict the published opponent velocity to the right-to-left
            # crossing through head-on sector of the GPS5->GPS6 route.
            ("clip_opponent_bearing_to_corridor", True),
            ("opponent_bearing_min_deg", -90.0),
            ("opponent_bearing_max_deg", 0.0),
        ])
        gp = lambda name: self.get_parameter(name).value  # noqa: E731

        self.map_frame = str(gp("map_frame"))
        self.base_frame = str(gp("base_frame"))
        self.opponent_frame = str(gp("opponent_frame"))
        self.policy = str(gp("selection_policy"))
        self.motion_filter_mode = str(gp("motion_filter_mode"))
        if self.motion_filter_mode not in ("standard", "straight_line"):
            raise ValueError(
                "motion_filter_mode must be 'standard' or 'straight_line'")
        straight_min_hit_count = int(gp("straight_min_hit_count"))
        straight_max_velocity_stddev_mps = float(
            gp("straight_max_velocity_stddev_mps"))
        if straight_min_hit_count < 1 or straight_max_velocity_stddev_mps <= 0.0:
            raise ValueError(
                "straight_min_hit_count must be >= 1 and "
                "straight_max_velocity_stddev_mps must be > 0")
        self.straight_coast_timeout_sec = float(gp("straight_coast_timeout_sec"))
        if self.straight_coast_timeout_sec < 0.0:
            raise ValueError("straight_coast_timeout_sec must be >= 0")
        self.corridor_enabled = bool(gp("corridor_enabled"))
        self.corridor_start_offset_m = float(gp("corridor_start_offset_m"))
        self.corridor_end_margin_m = float(gp("corridor_end_margin_m"))
        self.corridor_half_width_m = float(gp("corridor_half_width_m"))
        self.corridor_ignore_left_side = bool(gp("corridor_ignore_left_side"))
        self.corridor_left_side_margin_m = float(gp("corridor_left_side_margin_m"))
        self.clip_opponent_bearing_to_corridor = bool(
            gp("clip_opponent_bearing_to_corridor"))
        self.opponent_bearing_min_rad = math.radians(
            float(gp("opponent_bearing_min_deg")))
        self.opponent_bearing_max_rad = math.radians(
            float(gp("opponent_bearing_max_deg")))
        if self.corridor_start_offset_m < 0.0 or \
                self.corridor_end_margin_m < 0.0 or \
                self.corridor_half_width_m <= 0.0:
            raise ValueError("Task 2 corridor dimensions must be positive")
        if self.corridor_left_side_margin_m < 0.0:
            raise ValueError("corridor_left_side_margin_m must be >= 0")
        if not -math.pi <= self.opponent_bearing_min_rad <= \
                self.opponent_bearing_max_rad <= math.pi:
            raise ValueError("opponent bearing limits must be within [-180, 180] deg")
        self.corridor_start_map = None
        self.corridor_end_map = None
        self.selection_params = SelectionParams(
            confirmed_only=bool(gp("confirmed_only")),
            max_distance_m=float(gp("max_distance_m")),
            min_length_m=float(gp("min_length_m")),
            max_length_m=float(gp("max_length_m")),
            min_width_m=float(gp("min_width_m")),
            max_width_m=float(gp("max_width_m")),
            min_point_count=int(gp("min_point_count")),
            stale_timeout_sec=float(gp("stale_timeout_sec")),
            target_track_id=int(gp("target_track_id")),
            min_hit_count=(straight_min_hit_count
                           if self.motion_filter_mode == "straight_line" else 0),
            max_velocity_stddev_mps=(straight_max_velocity_stddev_mps
                                     if self.motion_filter_mode == "straight_line"
                                     else float("inf")),
        )
        self.smoother = TwistSmoother(
            alpha=float(gp("twist_lowpass_alpha")),
            max_speed_mps=knots_to_mps(gp("max_absolute_speed_knots")))
        self.min_absolute_speed_knots = float(gp("min_absolute_speed_knots"))
        self.max_absolute_speed_knots = float(gp("max_absolute_speed_knots"))
        if self.min_absolute_speed_knots < 0.0 or \
                self.max_absolute_speed_knots < self.min_absolute_speed_knots:
            raise ValueError(
                "Require 0 <= min_absolute_speed_knots <= "
                "max_absolute_speed_knots")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.pub = self.create_publisher(TwistStamped, str(gp("twist_topic")), 10)
        self.detection_status_pub = self.create_publisher(
            Bool, str(gp("detection_status_topic")), 10)
        self.selected_marker_pub = self.create_publisher(
            Marker, str(gp("selected_marker_topic")), 10)
        self.velocity_marker_pub = self.create_publisher(
            Marker, str(gp("velocity_marker_topic")), 10)
        self.create_subscription(
            TrackedObjectArray, str(gp("tracked_objects_topic")),
            self.tracks_callback, 10)
        self.create_subscription(
            Odometry, str(gp("ego_odom_topic")), self.ego_callback, 10)
        self.create_subscription(
            PoseStamped, str(gp("corridor_start_topic")),
            self.corridor_start_callback, 10)
        self.create_subscription(
            PoseStamped, str(gp("corridor_end_topic")),
            self.corridor_end_callback, 10)

        self.tracks: list[Track] = []
        self.ego_vel_base = np.zeros(3)  # ego twist linear, base_link (odom child frame)
        self.ego_yaw_rate = 0.0          # ego twist angular z, base_link [rad/s]
        self.selected_id = None
        # Last *measured* and fully gated target state.  It is used only by
        # the bounded straight-line coast path below, never as a replacement
        # for an unconfirmed detection.
        self.last_observation = None

        period = 1.0 / max(float(gp("publish_rate_hz")), 1e-6)
        self.timer = self.create_timer(period, self.timer_callback)
        self.get_logger().info(
            f"opponent_selector: policy={self.policy}, "
            f"motion_filter_mode={self.motion_filter_mode}, "
            f"tracks={gp('tracked_objects_topic')}, "
            f"ego_odom={gp('ego_odom_topic')} -> {gp('twist_topic')} "
            f"+ TF {self.map_frame} -> {self.opponent_frame}, "
            f"absolute_speed_range={self.min_absolute_speed_knots:.2f}"
            f"-{self.max_absolute_speed_knots:.2f} kn, "
            f"straight_coast={self.straight_coast_timeout_sec:.1f}s, "
            f"GPS5->6 corridor={'enabled' if self.corridor_enabled else 'disabled'}, "
            f"left_side_ignored={self.corridor_ignore_left_side}, "
            f"bearing_clip={'enabled' if self.clip_opponent_bearing_to_corridor else 'disabled'}")

    # ------------------------------------------------------------------
    def tracks_callback(self, msg):
        tracks = []
        for obj in msg.objects:
            stamp = obj.header.stamp.sec + obj.header.stamp.nanosec * 1e-9
            pos = obj.state.pose.pose.position
            vel = obj.state.twist.twist.linear
            twist_covariance = obj.state.twist.covariance
            tracks.append(Track(
                object_id=int(obj.object_id),
                position=np.array([pos.x, pos.y, pos.z]),
                yaw=_yaw_from_quaternion(obj.state.pose.pose.orientation),
                velocity_body=np.array([vel.x, vel.y, vel.z]),
                yaw_rate=float(obj.state.twist.twist.angular.z),
                dimensions=np.array(
                    [obj.dimensions.x, obj.dimensions.y, obj.dimensions.z]),
                point_count=int(obj.point_count),
                track_state=int(obj.track_state),
                stamp_sec=stamp,
                object_class=str(obj.object_class),
                hit_count=int(obj.hit_count),
                miss_count=int(obj.miss_count),
                velocity_stddev_mps=math.sqrt(max(
                    0.0, float(twist_covariance[0]), float(twist_covariance[7]))),
            ))
        self.tracks = tracks

    def ego_callback(self, msg: Odometry):
        lin = msg.twist.twist.linear
        self.ego_vel_base = np.array([lin.x, lin.y, lin.z])
        self.ego_yaw_rate = float(msg.twist.twist.angular.z)

    def _store_corridor_endpoint(self, msg: PoseStamped, endpoint: str):
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            self.get_logger().warning(
                f"Ignoring Task 2 corridor {endpoint} in frame "
                f"'{msg.header.frame_id}'; expected '{self.map_frame}'.",
                throttle_duration_sec=2.0)
            return
        point = np.array([msg.pose.position.x, msg.pose.position.y])
        if endpoint == "start":
            self.corridor_start_map = point
        else:
            self.corridor_end_map = point

    def corridor_start_callback(self, msg: PoseStamped):
        self._store_corridor_endpoint(msg, "start")

    def corridor_end_callback(self, msg: PoseStamped):
        self._store_corridor_endpoint(msg, "end")

    def _in_task2_corridor(self, position_map: np.ndarray) -> bool:
        if not self.corridor_enabled:
            return True
        if self.corridor_start_map is None or self.corridor_end_map is None:
            self.get_logger().warning(
                "No map-frame GPS5/GPS6 waypoints yet; not recognizing "
                "opponents until the Task 2 corridor is defined.",
                throttle_duration_sec=2.0)
            return False
        if not tracking_glue.in_oriented_corridor(
            position_map, self.corridor_start_map, self.corridor_end_map,
            self.corridor_start_offset_m, self.corridor_end_margin_m,
            self.corridor_half_width_m):
            return False
        return not (self.corridor_ignore_left_side and
                    tracking_glue.is_left_of_oriented_line(
                        position_map, self.corridor_start_map,
                        self.corridor_end_map,
                        self.corridor_left_side_margin_m))

    def _clip_opponent_velocity_bearing(
            self, velocity_map: np.ndarray) -> np.ndarray:
        if not self.clip_opponent_bearing_to_corridor or \
                self.corridor_start_map is None or self.corridor_end_map is None:
            return velocity_map
        route = self.corridor_end_map - self.corridor_start_map
        route_yaw = math.atan2(route[1], route[0])
        return tracking_glue.clip_velocity_bearing(
            velocity_map, route_yaw,
            self.opponent_bearing_min_rad, self.opponent_bearing_max_rad)

    def _publish_output(self, now, pos_map, vel_map, opponent_yaw, yaw_rate):
        """Publish one absolute target estimate and its map-frame TF."""
        smoothed = self.smoother.update(vel_map[0], vel_map[1], yaw_rate)
        if smoothed is None:
            self.get_logger().warning(
                "Opponent velocity spike rejected; skipping this cycle.",
                throttle_duration_sec=2.0)
            self._publish_detection_status(False)
            self._clear_velocity_marker(now)
            return False
        vx, vy, wz = smoothed

        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.map_frame
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.angular.z = float(wz)
        self.pub.publish(msg)

        tf_out = TransformStamped()
        tf_out.header.stamp = now.to_msg()
        tf_out.header.frame_id = self.map_frame
        tf_out.child_frame_id = self.opponent_frame
        tf_out.transform.translation.x = float(pos_map[0])
        tf_out.transform.translation.y = float(pos_map[1])
        tf_out.transform.translation.z = 0.0
        tf_out.transform.rotation.z = math.sin(opponent_yaw / 2.0)
        tf_out.transform.rotation.w = math.cos(opponent_yaw / 2.0)
        self.tf_broadcaster.sendTransform(tf_out)
        self._publish_detection_status(True)
        return True

    def _publish_detection_status(self, detected: bool):
        self.detection_status_pub.publish(Bool(data=detected))

    def _publish_selected_marker(self, now, position, yaw, dimensions):
        marker = Marker()
        marker.header.stamp = now.to_msg()
        marker.header.frame_id = self.map_frame
        marker.ns = "selected_opponent"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = position
        marker.pose.orientation.z = math.sin(yaw / 2.0)
        marker.pose.orientation.w = math.cos(yaw / 2.0)
        marker.scale.x, marker.scale.y, marker.scale.z = dimensions
        marker.color.g = 1.0
        marker.color.a = 0.85
        self.selected_marker_pub.publish(marker)

    def _publish_velocity_marker(self, now, position, velocity):
        """Publish the map-frame velocity vector of the selected opponent."""
        speed = float(np.hypot(velocity[0], velocity[1]))
        if speed <= 1e-6:
            self._clear_velocity_marker(now)
            return
        marker = Marker()
        marker.header.stamp = now.to_msg()
        marker.header.frame_id = self.map_frame
        marker.ns = "selected_opponent_velocity"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.12
        marker.scale.y = 0.24
        marker.scale.z = 0.30
        marker.color.r = 0.1
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 0.95
        start = Point(x=float(position[0]), y=float(position[1]),
                      z=float(position[2]) + 1.0)
        # Two metres of display arrow per metre/second preserves the vector
        # direction while keeping the expected Task 2 speed legible in 3D.
        end = Point(x=start.x + 2.0 * float(velocity[0]),
                    y=start.y + 2.0 * float(velocity[1]), z=start.z)
        marker.points = [start, end]
        self.velocity_marker_pub.publish(marker)

    def _clear_selected_marker(self, now):
        marker = Marker()
        marker.header.stamp = now.to_msg()
        marker.header.frame_id = self.map_frame
        marker.ns = "selected_opponent"
        marker.id = 0
        marker.action = Marker.DELETE
        self.selected_marker_pub.publish(marker)

    def _clear_velocity_marker(self, now):
        marker = Marker()
        marker.header.stamp = now.to_msg()
        marker.header.frame_id = self.map_frame
        marker.ns = "selected_opponent_velocity"
        marker.id = 0
        marker.action = Marker.DELETE
        self.velocity_marker_pub.publish(marker)

    def _coast_or_silence(self, now, now_sec: float):
        """Bridge brief occlusions with constant-velocity prediction only."""
        coast = self.last_observation
        if self.motion_filter_mode == "straight_line" and coast is not None:
            elapsed = now_sec - coast["stamp_sec"]
            if 0.0 <= elapsed <= self.straight_coast_timeout_sec:
                pos_map = tracking_glue.predict_straight_motion(
                    coast["position_map"], coast["velocity_map"], elapsed)
                if not self._in_task2_corridor(pos_map):
                    self.last_observation = None
                    self.selected_id = None
                    self.smoother.reset()
                    self._clear_selected_marker(now)
                    self._clear_velocity_marker(now)
                    self._publish_detection_status(False)
                    return
                if self._publish_output(
                        now, pos_map, coast["velocity_map"], coast["yaw_map"],
                        coast["yaw_rate"]):
                    self._publish_velocity_marker(
                        now, pos_map, coast["velocity_map"])
                self.get_logger().info(
                    f"Coasting confirmed opponent id={coast['object_id']} "
                    f"for {elapsed:.1f}s without a LiDAR observation.",
                    throttle_duration_sec=1.0)
                return

        # Safe silence: MPPI plans a straight path without the opponent.
        self.get_logger().warning(
            "No valid opponent track or bounded straight-line coast. "
            "Publishing nothing on /other_ship/twist and no opponent TF.",
            throttle_duration_sec=2.0)
        self.selected_id = None
        self.last_observation = None
        self.smoother.reset()
        self._clear_selected_marker(now)
        self._clear_velocity_marker(now)
        self._publish_detection_status(False)

    # ------------------------------------------------------------------
    def timer_callback(self):
        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9

        ranked = tracking_glue.select_opponent(
            self.tracks, now_sec, policy=self.policy,
            params=self.selection_params)
        if not ranked:
            self._coast_or_silence(now, now_sec)
            return
        selected = None
        tf_msg = None
        pos_map = None
        vel_map = None
        for candidate in ranked:
            try:
                # Look up map<-base_link at the track's own stamp so an
                # up-to-stale_timeout_sec old track is composed with the
                # matching ego pose, not the latest one; fall back to latest.
                try:
                    candidate_tf = self.tf_buffer.lookup_transform(
                        self.map_frame, self.base_frame,
                        Time(seconds=candidate.stamp_sec))
                except TransformException:
                    candidate_tf = self.tf_buffer.lookup_transform(
                        self.map_frame, self.base_frame, Time())
            except TransformException as e:
                self.get_logger().warning(
                    f"TF {self.base_frame} -> {self.map_frame} unavailable: {e}",
                    throttle_duration_sec=2.0)
                self._publish_detection_status(False)
                return

            q = candidate_tf.transform.rotation
            t = candidate_tf.transform.translation
            t_map_base = cloud_ops.make_transform(
                cloud_ops.quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w),
                [t.x, t.y, t.z])

            # The tracker reports a body-frame relative velocity.  Restore
            # own-ship motion before applying the absolute-speed gate.
            vel_abs_base = tracking_glue.ego_compensate(
                candidate.velocity_base, self.ego_vel_base,
                ego_yaw_rate=self.ego_yaw_rate, pos_base=candidate.position)
            candidate_pos_map, candidate_vel_map = tracking_glue.to_map_frame(
                candidate.position, vel_abs_base, t_map_base)
            if not self._in_task2_corridor(candidate_pos_map):
                continue
            candidate_vel_map = self._clip_opponent_velocity_bearing(
                candidate_vel_map)
            speed_knots = mps_to_knots(
                np.hypot(candidate_vel_map[0], candidate_vel_map[1]))
            if not self.min_absolute_speed_knots <= speed_knots <= \
                    self.max_absolute_speed_knots:
                continue

            selected = candidate
            tf_msg = candidate_tf
            pos_map = candidate_pos_map
            vel_map = candidate_vel_map
            break

        if selected is None:
            self._coast_or_silence(now, now_sec)
            return

        if self.selected_id != selected.object_id:
            self.smoother.reset()
            self.selected_id = selected.object_id
            self.get_logger().info(
                f"Selected moving opponent track id={selected.object_id} "
                f"(dist={selected.distance:.1f} m)")

        q = tf_msg.transform.rotation
        base_yaw = _yaw_from_quaternion(q)
        opponent_yaw = base_yaw + selected.yaw
        self.last_observation = {
            "object_id": selected.object_id,
            "stamp_sec": now_sec,
            "position_map": pos_map.copy(),
            "velocity_map": vel_map.copy(),
            "yaw_map": opponent_yaw,
            "yaw_rate": selected.yaw_rate,
        }
        if self._publish_output(
                now, pos_map, vel_map, opponent_yaw, selected.yaw_rate):
            self._publish_selected_marker(
                now, pos_map, opponent_yaw, selected.dimensions)
            self._publish_velocity_marker(now, pos_map, vel_map)


def main(args=None):
    if TrackedObjectArray is None:
        rclpy.logging.get_logger("opponent_selector").fatal(
            "Cannot import ship_perception_msgs (TrackedObjectArray). "
            "This interface package is provided by the pcl_segmentation "
            "submodule — build it first (colcon build --packages-up-to "
            "ship_perception_msgs) and source the workspace.")
        raise SystemExit(1)
    rclpy.init(args=args)
    node = OpponentSelectorNode()
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
