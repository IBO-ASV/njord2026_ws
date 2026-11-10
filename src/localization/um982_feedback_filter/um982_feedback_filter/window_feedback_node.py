#!/usr/bin/env python3
"""Estimate body velocity from a time window of UM982 local positions."""

from collections import deque
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def regression_slope(samples, index):
    """Return least-squares slope for (time, x, y, yaw) samples."""
    if len(samples) < 2:
        return 0.0
    mean_t = sum(sample[0] for sample in samples) / len(samples)
    mean_value = sum(sample[index] for sample in samples) / len(samples)
    numerator = sum((sample[0] - mean_t) * (sample[index] - mean_value) for sample in samples)
    denominator = sum((sample[0] - mean_t) ** 2 for sample in samples)
    return numerator / denominator if denominator > 1e-9 else 0.0


class WindowFeedbackNode(Node):
    def __init__(self):
        super().__init__('um982_window_feedback')
        self.input_topic = self.declare_parameter('input_topic', '/odometry/feedback').value
        self.output_topic = self.declare_parameter('output_topic', '/odometry/filtered/local').value
        self.window_sec = self.declare_parameter('window_sec', 2.0).value
        self.linear_deadband_mps = self.declare_parameter('linear_deadband_mps', 0.10).value
        self.yaw_rate_deadband_rps = self.declare_parameter('yaw_rate_deadband_rps', 0.05).value
        self.samples = deque()
        self.previous_yaw = None
        self.unwrapped_yaw = 0.0
        self.last_stamp_sec = None
        self.publisher = self.create_publisher(Odometry, self.output_topic, 10)
        self.subscription = self.create_subscription(Odometry, self.input_topic, self.callback, 20)

    @staticmethod
    def deadband(value, threshold):
        return 0.0 if abs(value) < threshold else value

    def callback(self, message):
        stamp_sec = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if self.last_stamp_sec is not None and stamp_sec <= self.last_stamp_sec:
            return
        yaw = yaw_from_quaternion(message.pose.pose.orientation)
        if self.previous_yaw is None:
            self.unwrapped_yaw = yaw
        else:
            self.unwrapped_yaw += math.atan2(math.sin(yaw - self.previous_yaw), math.cos(yaw - self.previous_yaw))
        self.previous_yaw = yaw
        self.last_stamp_sec = stamp_sec
        self.samples.append((stamp_sec, message.pose.pose.position.x, message.pose.pose.position.y, self.unwrapped_yaw))
        while self.samples and stamp_sec - self.samples[0][0] > self.window_sec:
            self.samples.popleft()

        east_velocity = regression_slope(self.samples, 1)
        north_velocity = regression_slope(self.samples, 2)
        surge = math.cos(yaw) * east_velocity + math.sin(yaw) * north_velocity
        sway = -math.sin(yaw) * east_velocity + math.cos(yaw) * north_velocity
        yaw_rate = regression_slope(self.samples, 3)

        output = message
        output.twist.twist.linear.x = self.deadband(surge, self.linear_deadband_mps)
        output.twist.twist.linear.y = self.deadband(sway, self.linear_deadband_mps)
        output.twist.twist.angular.z = self.deadband(yaw_rate, self.yaw_rate_deadband_rps)
        output.twist.covariance[0] = 0.04
        output.twist.covariance[7] = 0.04
        output.twist.covariance[35] = 0.01
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = WindowFeedbackNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
