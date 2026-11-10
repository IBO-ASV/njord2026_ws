#!/usr/bin/env python3
"""Publish the sole Task 2 readiness state for FollowPath-only bringup.

Task 2 keeps the generic ``autonomy_supervisor`` disabled, so this node owns
``/autonomy/ready`` while the persistent Mission/Control graph is active.
"""

from __future__ import annotations

import rclpy
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool


class Task2AutonomyReadyNode(Node):
    def __init__(self) -> None:
        super().__init__("task2_autonomy_ready")

        self.declare_parameter("path_topic", "/planned_path_pruned")
        self.declare_parameter("action_name", "/follow_path")
        self.declare_parameter("ready_topic", "/autonomy/ready")
        self.declare_parameter("path_timeout_sec", 2.0)
        self.declare_parameter("publish_frequency_hz", 5.0)

        path_topic = str(self.get_parameter("path_topic").value)
        action_name = str(self.get_parameter("action_name").value)
        ready_topic = str(self.get_parameter("ready_topic").value)
        self.path_timeout_sec = float(self.get_parameter("path_timeout_sec").value)
        frequency = float(self.get_parameter("publish_frequency_hz").value)

        self.last_valid_path_time = None
        self.path_sub = self.create_subscription(Path, path_topic, self._path_callback, 10)
        self.ready_pub = self.create_publisher(Bool, ready_topic, 10)
        self.follow_path_client = ActionClient(self, FollowPath, action_name)
        self.timer = self.create_timer(1.0 / max(frequency, 1e-3), self._publish_ready)

        self.get_logger().info(
            f"Task2 readiness requires {path_topic} and FollowPath action {action_name}"
        )

    def _path_callback(self, msg: Path) -> None:
        if len(msg.poses) >= 2:
            self.last_valid_path_time = self.get_clock().now()

    def _publish_ready(self) -> None:
        path_fresh = False
        if self.last_valid_path_time is not None:
            age_sec = (self.get_clock().now() - self.last_valid_path_time).nanoseconds * 1e-9
            path_fresh = age_sec <= self.path_timeout_sec

        ready = path_fresh and self.follow_path_client.server_is_ready()
        self.ready_pub.publish(Bool(data=ready))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Task2AutonomyReadyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
