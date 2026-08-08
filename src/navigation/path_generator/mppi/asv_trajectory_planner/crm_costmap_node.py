#!/usr/bin/env python3
"""Publish the MPPI Collision Risk Model as a Nav2-style rolling costmap."""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener

from asv_trajectory_planner import crm_torch


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _rotate_xy(x, y, q):
    c, s = math.cos(_yaw(q)), math.sin(_yaw(q))
    return c * x - s * y, s * x + c * y


class CrmCostmapNode(Node):
    def __init__(self):
        super().__init__("crm_costmap")
        self.declare_parameters("", [
            ("own_odom_topic", "/odom"), ("other_ship_twist_topic", "/other_ship/twist"),
            ("costmap_topic", "/mppi/crm_costmap"), ("costmap_frame", "odom"),
            ("base_frame", "base_link"), ("opponent_frame", "opponent_vessel"),
            # Match Task2 Nav2 local_costmap geometry.
            ("width_m", 30.0), ("height_m", 30.0), ("resolution_m", 0.1),
            ("rolling_window", True), ("origin_x_m", -20.0), ("origin_y_m", -30.0),
            ("publish_rate_hz", 2.0), ("prediction_horizon_sec", 22.5),
            ("prediction_step_sec", 0.5), ("loa_m", 2.0),
        ])
        p = lambda n: self.get_parameter(n).value
        self.frame, self.base, self.opponent = map(str, (p("costmap_frame"), p("base_frame"), p("opponent_frame")))
        self.width, self.height, self.resolution = float(p("width_m")), float(p("height_m")), float(p("resolution_m"))
        self.rolling_window = bool(p("rolling_window")); self.origin_x, self.origin_y = float(p("origin_x_m")), float(p("origin_y_m"))
        self.cols, self.rows = round(self.width / self.resolution), round(self.height / self.resolution)
        self.horizon, self.step, self.loa = float(p("prediction_horizon_sec")), float(p("prediction_step_sec")), float(p("loa_m"))
        self.odom = self.twist = None
        self.tf_buffer = Buffer(); self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, str(p("own_odom_topic")), self._odom, 10)
        self.create_subscription(TwistStamped, str(p("other_ship_twist_topic")), self._twist, 10)
        self.pub = self.create_publisher(OccupancyGrid, str(p("costmap_topic")), 1)
        self.create_timer(1.0 / max(0.1, float(p("publish_rate_hz"))), self._publish)

    def _odom(self, msg): self.odom = msg
    def _twist(self, msg): self.twist = msg

    def _publish(self):
        if self.odom is None or self.twist is None:
            return
        try:
            own_tf = self.tf_buffer.lookup_transform(self.frame, self.base, rclpy.time.Time(), timeout=Duration(seconds=0.05))
            oth_tf = self.tf_buffer.lookup_transform(self.frame, self.opponent, rclpy.time.Time(), timeout=Duration(seconds=0.05))
            vel_tf = self.tf_buffer.lookup_transform(self.frame, self.twist.header.frame_id or self.frame, rclpy.time.Time(), timeout=Duration(seconds=0.05))
        except TransformException as exc:
            self.get_logger().warning(f"CRM costmap TF unavailable: {exc}", throttle_duration_sec=2.0)
            return

        ox, oy = own_tf.transform.translation.x, own_tf.transform.translation.y
        tx, ty = oth_tf.transform.translation.x, oth_tf.transform.translation.y
        vx, vy = _rotate_xy(self.twist.twist.linear.x, self.twist.twist.linear.y, vel_tf.transform.rotation)
        own_speed = math.hypot(self.odom.twist.twist.linear.x, self.odom.twist.twist.linear.y)
        own_heading = (-math.degrees(_yaw(own_tf.transform.rotation))) % 360.0
        target_heading = (-math.degrees(math.atan2(vy, vx))) % 360.0 if math.hypot(vx, vy) > 1e-4 else (-math.degrees(_yaw(oth_tf.transform.rotation))) % 360.0

        origin_x = ox - self.width / 2 if self.rolling_window else self.origin_x
        origin_y = oy - self.height / 2 if self.rolling_window else self.origin_y
        xs = origin_x + (np.arange(self.cols) + 0.5) * self.resolution
        ys = origin_y + (np.arange(self.rows) + 0.5) * self.resolution
        grid_x, grid_y = np.meshgrid(xs, ys)
        own = [-oy, ox, own_speed, own_heading, self.loa]
        other = [[-ty, tx, math.hypot(vx, vy), target_heading, self.loa]]
        risk = np.zeros_like(grid_x)
        for t in np.arange(0.0, self.horizon + 1e-9, self.step):
            risk = np.maximum(risk, crm_torch.timedomaincrm_numpy(-grid_y, grid_x, t, own, other))
        risk = np.nan_to_num(risk, nan=0.0, posinf=1.0, neginf=0.0)

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg(); msg.header.frame_id = self.frame
        msg.info.resolution = self.resolution; msg.info.width = self.cols; msg.info.height = self.rows
        msg.info.origin.position.x = origin_x; msg.info.origin.position.y = origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = np.rint(np.clip(risk, 0.0, 1.0) * 100.0).astype(np.int8).ravel().tolist()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args); node = CrmCostmapNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
