#!/usr/bin/env python3
"""Relay Task 2 waypoints into map without changing the navigation topics."""

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

# Registers the geometry message conversions used by Buffer.transform().
import tf2_geometry_msgs  # noqa: F401

try:
    from tf2_ros import TransformException
except ImportError:
    from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
    TransformException = (LookupException, ConnectivityException, ExtrapolationException)


class Task2WaypointMapFrameRelay(Node):
    """Publish map-frame copies of the odom-frame navigation waypoints.

    A missing transform deliberately produces no output.  Consumers therefore
    cannot silently apply GPS5->6 geometry to coordinates in the wrong frame.
    """

    def __init__(self):
        super().__init__("task2_waypoint_map_frame_relay")
        self.declare_parameters("", [
            ("waypoint1_input_topic", "/waypoint1_pose"),
            ("waypoint2_input_topic", "/waypoint2_pose"),
            ("waypoint1_output_topic", "/task2/waypoint1_pose_map"),
            ("waypoint2_output_topic", "/task2/waypoint2_pose_map"),
            ("target_frame", "map"),
            ("transform_timeout_sec", 0.2),
        ])
        p = lambda name: self.get_parameter(name).value
        self.target_frame = str(p("target_frame"))
        self.timeout = Duration(seconds=float(p("transform_timeout_sec")))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.waypoint1_pub = self.create_publisher(
            PoseStamped, str(p("waypoint1_output_topic")), 10)
        self.waypoint2_pub = self.create_publisher(
            PoseStamped, str(p("waypoint2_output_topic")), 10)
        self.create_subscription(
            PoseStamped, str(p("waypoint1_input_topic")),
            lambda msg: self._relay(msg, self.waypoint1_pub, "GPS5"), 10)
        self.create_subscription(
            PoseStamped, str(p("waypoint2_input_topic")),
            lambda msg: self._relay(msg, self.waypoint2_pub, "GPS6"), 10)

    def _relay(self, msg: PoseStamped, publisher, label: str):
        if not msg.header.frame_id:
            self.get_logger().warning(
                f"Not relaying {label}: waypoint frame is empty.",
                throttle_duration_sec=2.0)
            return
        try:
            transformed = self.tf_buffer.transform(
                msg, self.target_frame, timeout=self.timeout)
        except TransformException as error:
            self.get_logger().warning(
                f"Not relaying {label} to {self.target_frame}: {error}",
                throttle_duration_sec=2.0)
            return
        publisher.publish(transformed)


def main(args=None):
    rclpy.init(args=args)
    node = Task2WaypointMapFrameRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
