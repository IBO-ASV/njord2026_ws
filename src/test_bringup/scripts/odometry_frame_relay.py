#!/usr/bin/env python3
"""Republish odometry using a configured parent frame.

This is intended for offline replay where navsat_transform_node publishes a
local tangent-plane position labelled ``odom`` but the global EKF needs an
initial map-frame observation in order to estimate ``map -> odom``.  It does
not apply a spatial transform: the configured target frame declares that the
two origins are initially coincident for this replay experiment.
"""

from copy import deepcopy

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OdometryFrameRelay(Node):
    def __init__(self):
        super().__init__("odometry_frame_relay")
        self.declare_parameter("input_topic", "/playback/odometry/gps/um982")
        self.declare_parameter("output_topic", "/playback/odometry/gps/map")
        self.declare_parameter("target_frame", "map")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self._target_frame = self.get_parameter("target_frame").value
        self._publisher = self.create_publisher(Odometry, output_topic, 10)
        self._subscription = self.create_subscription(
            Odometry, input_topic, self._relay, 10
        )

    def _relay(self, message):
        relayed = deepcopy(message)
        relayed.header.frame_id = self._target_frame
        self._publisher.publish(relayed)


def main():
    rclpy.init()
    node = OdometryFrameRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
