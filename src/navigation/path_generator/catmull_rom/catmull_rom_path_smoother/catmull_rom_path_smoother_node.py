import math

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from .catmull_rom import generate_catmull_rom_path


def _yaw_to_quaternion(yaw):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class CatmullRomPathSmoother(Node):
    def __init__(self):
        super().__init__("catmull_rom_path_smoother")

        self.declare_parameter("input_topic", "/plan")
        self.declare_parameter("output_topic", "/plan_smoothed")
        self.declare_parameter("frame_id", "")
        self.declare_parameter("sample_interval", 0.5)
        self.declare_parameter("publish_transient_local", True)
        self.declare_parameter("preserve_input_header_stamp", False)
        self.declare_parameter("minimum_input_poses", 2)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.sample_interval = float(self.get_parameter("sample_interval").value)
        self.preserve_input_header_stamp = bool(
            self.get_parameter("preserve_input_header_stamp").value
        )
        self.minimum_input_poses = int(self.get_parameter("minimum_input_poses").value)

        qos = self._make_output_qos()
        self.publisher = self.create_publisher(Path, self.output_topic, qos)
        self.subscription = self.create_subscription(
            Path,
            self.input_topic,
            self._on_path,
            10,
        )

        self.get_logger().info(
            "Catmull-Rom path smoother started: "
            f"{self.input_topic} -> {self.output_topic}, "
            f"sample_interval={self.sample_interval:.3f}"
        )

    def _make_output_qos(self):
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        if bool(self.get_parameter("publish_transient_local").value):
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        return qos

    def _on_path(self, msg):
        if len(msg.poses) < self.minimum_input_poses:
            self.get_logger().warn(
                f"Received path with {len(msg.poses)} poses. "
                f"Need at least {self.minimum_input_poses}."
            )
            return

        points = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        smoothed_points = generate_catmull_rom_path(points, self.sample_interval)
        smoothed = self._points_to_path(smoothed_points, msg)
        self.publisher.publish(smoothed)

    def _points_to_path(self, points, source_msg):
        msg = Path()
        msg.header.frame_id = self.frame_id or source_msg.header.frame_id
        if self.preserve_input_header_stamp:
            msg.header.stamp = source_msg.header.stamp
        else:
            msg.header.stamp = self.get_clock().now().to_msg()

        for index, point in enumerate(points):
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.position.z = 0.0
            pose.pose.orientation = _yaw_to_quaternion(self._path_yaw(points, index))
            msg.poses.append(pose)

        return msg

    @staticmethod
    def _path_yaw(points, index):
        if len(points) < 2:
            return 0.0
        if index < len(points) - 1:
            dx = points[index + 1][0] - points[index][0]
            dy = points[index + 1][1] - points[index][1]
        else:
            dx = points[index][0] - points[index - 1][0]
            dy = points[index][1] - points[index - 1][1]
        return math.atan2(dy, dx)


def main(args=None):
    rclpy.init(args=args)
    node = CatmullRomPathSmoother()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
