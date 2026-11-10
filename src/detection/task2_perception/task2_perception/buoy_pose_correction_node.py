#!/usr/bin/env python3
"""Fuse stable Task 2 buoy-pair geometry into the map-frame EKF.

The red/green labels are a route-side association, not visual colours.  A
same-station red/green pair has its midpoint on the GPS5->GPS6 centre line;
its cross-track error and lateral direction therefore provide a bounded pose
observation.  This node publishes only a heavily gated measurement.
"""

import math
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

from njord_interfaces.msg import BuoyDetection, BuoyDetectionArray


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class BuoyPoseCorrection(Node):
    def __init__(self):
        super().__init__("task2_buoy_pose_correction")
        self.declare_parameters("", [
            ("buoy_topic", "/task2/buoy_detections"),
            ("odom_topic", "/odometry/filtered/global"),
            ("output_topic", "/task2/buoy_pose_correction"),
            ("waypoint_start_topic", "/waypoint1_pose"),
            ("waypoint_end_topic", "/waypoint2_pose"),
            ("frame_id", "map"),
            ("pair_separation_m", 5.0),
            ("pair_separation_tolerance_m", 2.0),
            ("max_pair_along_error_m", 2.0),
            ("max_cross_track_correction_m", 2.0),
            ("max_yaw_correction_rad", 0.35),
            ("stable_samples_required", 5),
            ("position_stddev_m", 0.5),
            ("along_track_stddev_m", 20.0),
            ("yaw_stddev_rad", 0.12),
        ])
        p = lambda name: self.get_parameter(name).value
        self.frame_id = str(p("frame_id"))
        self.pair_separation_m = float(p("pair_separation_m"))
        self.pair_separation_tolerance_m = float(p("pair_separation_tolerance_m"))
        self.max_pair_along_error_m = float(p("max_pair_along_error_m"))
        self.max_cross_track_correction_m = float(p("max_cross_track_correction_m"))
        self.max_yaw_correction_rad = float(p("max_yaw_correction_rad"))
        self.required = int(p("stable_samples_required"))
        self.position_stddev_m = float(p("position_stddev_m"))
        self.along_track_stddev_m = float(p("along_track_stddev_m"))
        self.yaw_stddev_rad = float(p("yaw_stddev_rad"))
        self.start = self.end = self.odom = None
        self.samples = deque(maxlen=max(1, self.required))
        self.pub = self.create_publisher(PoseWithCovarianceStamped, str(p("output_topic")), 10)
        self.create_subscription(BuoyDetectionArray, str(p("buoy_topic")), self.on_buoys, 10)
        self.create_subscription(Odometry, str(p("odom_topic")), self.on_odom, 10)
        self.create_subscription(PoseStamped, str(p("waypoint_start_topic")), lambda msg: self.on_waypoint(msg, True), 10)
        self.create_subscription(PoseStamped, str(p("waypoint_end_topic")), lambda msg: self.on_waypoint(msg, False), 10)

    def on_odom(self, msg):
        if msg.header.frame_id == self.frame_id:
            self.odom = msg

    def on_waypoint(self, msg, is_start):
        if msg.header.frame_id and msg.header.frame_id != self.frame_id:
            return
        point = (msg.pose.position.x, msg.pose.position.y)
        if is_start:
            self.start = point
        else:
            self.end = point

    def on_buoys(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.frame_id:
            return
        if self.start is None or self.end is None or self.odom is None:
            return
        red = [d.position for d in msg.detections if d.class_id == BuoyDetection.CLASS_RED]
        green = [d.position for d in msg.detections if d.class_id == BuoyDetection.CLASS_GREEN]
        if not red or not green:
            self.samples.clear()
            return
        dx, dy = self.end[0] - self.start[0], self.end[1] - self.start[1]
        length = math.hypot(dx, dy)
        if length < 1e-3:
            return
        ux, uy = dx / length, dy / length
        nx, ny = -uy, ux
        pair = min(((r, g) for r in red for g in green), key=lambda rg: math.hypot(rg[0].x-rg[1].x, rg[0].y-rg[1].y))
        r, g = pair
        vx, vy = g.x-r.x, g.y-r.y
        separation = math.hypot(vx, vy)
        along_error = abs(vx * ux + vy * uy)
        if abs(separation - self.pair_separation_m) > self.pair_separation_tolerance_m or along_error > self.max_pair_along_error_m:
            self.samples.clear()
            return
        midpoint = ((r.x + g.x) * 0.5, (r.y + g.y) * 0.5)
        cross_error = (midpoint[0] - self.start[0]) * nx + (midpoint[1] - self.start[1]) * ny
        observed_pair_yaw = math.atan2(vy, vx)
        expected_pair_yaw = math.atan2(-ny, -nx)  # red (+normal) -> green (-normal)
        yaw_error = _wrap(observed_pair_yaw - expected_pair_yaw)
        if abs(cross_error) > self.max_cross_track_correction_m or abs(yaw_error) > self.max_yaw_correction_rad:
            self.samples.clear()
            return
        pose = self.odom.pose.pose
        self.samples.append((cross_error, yaw_error))
        if len(self.samples) < self.required:
            return
        cross = sum(sample[0] for sample in self.samples) / len(self.samples)
        yaw = _yaw(pose.orientation) - sum(sample[1] for sample in self.samples) / len(self.samples)
        out = PoseWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.pose.pose.position.x = pose.position.x - cross * nx
        out.pose.pose.position.y = pose.position.y - cross * ny
        out.pose.pose.orientation.z = math.sin(yaw * 0.5)
        out.pose.pose.orientation.w = math.cos(yaw * 0.5)
        # High uncertainty along the course prevents this line constraint from
        # pretending it knows where between GPS5 and GPS6 the vessel is.
        along_var, cross_var = self.along_track_stddev_m**2, self.position_stddev_m**2
        out.pose.covariance[0] = along_var * ux * ux + cross_var * nx * nx
        out.pose.covariance[1] = along_var * ux * uy + cross_var * nx * ny
        out.pose.covariance[6] = out.pose.covariance[1]
        out.pose.covariance[7] = along_var * uy * uy + cross_var * ny * ny
        out.pose.covariance[14] = 1e6
        out.pose.covariance[21] = 1e6
        out.pose.covariance[28] = 1e6
        out.pose.covariance[35] = self.yaw_stddev_rad**2
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = BuoyPoseCorrection()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
