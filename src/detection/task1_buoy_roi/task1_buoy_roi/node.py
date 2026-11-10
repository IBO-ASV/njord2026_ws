"""Task 1 fallback LiDAR ROI from surveyed buoy positions."""

from nav_msgs.msg import Odometry
from njord_interfaces.msg import BuoyRoi
import rclpy
from rclpy.node import Node

from task1_buoy_roi.geometry import nearest_roi, parse_positions, yaw_from_quaternion


DEFAULT_BUOY_POSITIONS = "[[40.0, -25.0], [28.0, -25.0], [18.0, -25.0], [11.0, -25.0]]"


class Task1BuoyRoi(Node):
    def __init__(self):
        super().__init__("task1_buoy_roi")
        self.declare_parameter("odometry_topic", "/odometry/filtered/global")
        self.declare_parameter("output_topic", "/buoy_roi")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("buoy_position_xy", DEFAULT_BUOY_POSITIONS)
        self.declare_parameter("activation_range_m", 10.0)
        self.declare_parameter("range_half_width_m", 2.0)
        self.declare_parameter("bearing_half_width_rad", 0.35)
        self.positions = parse_positions(self.get_parameter("buoy_position_xy").value)
        if not self.positions:
            raise ValueError("buoy_position_xy must be a JSON array of [x, y] map positions")
        self.map_frame = self.get_parameter("map_frame").value
        self.activation_range = max(0.0, float(self.get_parameter("activation_range_m").value))
        self.range_half_width = max(0.0, float(self.get_parameter("range_half_width_m").value))
        self.bearing_half_width = max(0.0, float(self.get_parameter("bearing_half_width_rad").value))
        self.publisher = self.create_publisher(BuoyRoi, self.get_parameter("output_topic").value, 10)
        self.create_subscription(
            Odometry, self.get_parameter("odometry_topic").value, self.on_odometry, 10)

    def on_odometry(self, odometry):
        if odometry.header.frame_id and odometry.header.frame_id != self.map_frame:
            self.get_logger().warn(
                f"Ignoring odometry in {odometry.header.frame_id}; expected {self.map_frame}",
                throttle_duration_sec=5.0)
            return
        pose = odometry.pose.pose
        roi = nearest_roi(
            pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation),
            self.positions, self.activation_range)
        if roi is None:
            return
        message = BuoyRoi()
        message.header = odometry.header
        message.header.frame_id = "base_link"
        message.r_predict, message.theta_predict = roi
        message.r_range = self.range_half_width
        message.theta_range = self.bearing_half_width
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = Task1BuoyRoi()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
