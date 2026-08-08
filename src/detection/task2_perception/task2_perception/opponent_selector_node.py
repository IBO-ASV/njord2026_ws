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

from geometry_msgs.msg import TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

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
        self.create_subscription(
            TrackedObjectArray, str(gp("tracked_objects_topic")),
            self.tracks_callback, 10)
        self.create_subscription(
            Odometry, str(gp("ego_odom_topic")), self.ego_callback, 10)

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
            f"straight_coast={self.straight_coast_timeout_sec:.1f}s")

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

    def _publish_output(self, now, pos_map, vel_map, opponent_yaw, yaw_rate):
        """Publish one absolute target estimate and its map-frame TF."""
        smoothed = self.smoother.update(vel_map[0], vel_map[1], yaw_rate)
        if smoothed is None:
            self.get_logger().warning(
                "Opponent velocity spike rejected; skipping this cycle.",
                throttle_duration_sec=2.0)
            return
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

    def _coast_or_silence(self, now, now_sec: float):
        """Bridge brief occlusions with constant-velocity prediction only."""
        coast = self.last_observation
        if self.motion_filter_mode == "straight_line" and coast is not None:
            elapsed = now_sec - coast["stamp_sec"]
            if 0.0 <= elapsed <= self.straight_coast_timeout_sec:
                pos_map = tracking_glue.predict_straight_motion(
                    coast["position_map"], coast["velocity_map"], elapsed)
                self._publish_output(
                    now, pos_map, coast["velocity_map"], coast["yaw_map"],
                    coast["yaw_rate"])
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
        self._publish_output(
            now, pos_map, vel_map, opponent_yaw, selected.yaw_rate)


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
        rclpy.shutdown()


if __name__ == "__main__":
    main()
