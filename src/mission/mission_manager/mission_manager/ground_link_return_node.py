"""ROS adapter that turns a stable 30 s ground-link loss into one mission request."""

from __future__ import annotations

import math
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Empty, String

from .geodesy import load_home_datum, yaw_from_quaternion
from .ground_link_failsafe import GroundLinkReturnEvaluator


TRANSIENT_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class GroundLinkReturnNode(Node):
    def __init__(self) -> None:
        super().__init__("ground_link_return_monitor")
        default_home = str(Path(get_package_share_directory("mission_manager")) / "config" / "home.yaml")
        home_file = Path(self.declare_parameter("home_file", default_home).value)
        self._evaluator = GroundLinkReturnEvaluator(load_home_datum(home_file))
        self._last_status = ""
        self._return_pub = self.create_publisher(Empty, "/mission/failsafe/return_home", TRANSIENT_QOS)
        self._status_pub = self.create_publisher(String, "/mission/failsafe/status", TRANSIENT_QOS)
        self.create_subscription(Empty, "/heartbeat/ground_station", self._heartbeat, 10)
        self.create_subscription(Twist, "/cmd_vel_sbus", self._sbus_command, 10)
        self.create_subscription(NavSatFix, "/sensor/vehicle_gnss/fix/raw", self._position, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/sensor/vehicle_gnss/compass/raw", self._heading, 10)
        self.create_timer(0.1, self._tick)
        self._publish_status("ground-link return monitor waiting for first heartbeat")

    @staticmethod
    def _now() -> float:
        return time.monotonic()

    def _heartbeat(self, _message: Empty) -> None:
        self._evaluator.heartbeat(self._now())
        self._publish_status("ground heartbeat healthy")

    def _sbus_command(self, _message: Twist) -> None:
        self._evaluator.heartbeat(self._now())
        self._publish_status("SBUS command active; ground heartbeat bypassed")

    def _position(self, message: NavSatFix) -> None:
        if math.isfinite(message.latitude) and math.isfinite(message.longitude):
            self._evaluator.position(self._now(), message.latitude, message.longitude)

    def _heading(self, message: PoseWithCovarianceStamped) -> None:
        yaw = yaw_from_quaternion(message.pose.pose.orientation)
        if math.isfinite(yaw):
            self._evaluator.heading(self._now(), yaw)

    def _tick(self) -> None:
        decision = self._evaluator.evaluate(self._now())
        if decision is None:
            return
        self._publish_status(decision.status)
        if decision.trigger_return_home:
            self._return_pub.publish(Empty())

    def _publish_status(self, status: str) -> None:
        if status != self._last_status:
            self._last_status = status
            self._status_pub.publish(String(data=status))
            self.get_logger().info(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundLinkReturnNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
