"""Forward one fresh command source, or an explicit zero command."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from njord_interfaces.msg import ControlState

from .arbitration import Source, compatibility_control_status, select_source


TRANSIENT_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class CommandArbiter(Node):
    """Does not evaluate health policy; it only consumes canonical control state."""

    def __init__(self) -> None:
        super().__init__("command_arbiter")
        self._command_timeout_sec = float(self.declare_parameter("command_timeout_sec", 2.0).value)
        self._publish_period_sec = float(self.declare_parameter("publish_period_sec", 0.05).value)
        if self._command_timeout_sec <= 0 or self._publish_period_sec <= 0:
            raise ValueError("command_timeout_sec and publish_period_sec must be positive")
        self._state: ControlState | None = None
        self._manual_command = Twist()
        self._nav_command = Twist()
        self._sbus_command = Twist()
        self._manual_received_ns: int | None = None
        self._nav_received_ns: int | None = None
        self._sbus_received_ns: int | None = None
        self._command_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._control_status_pub = self.create_publisher(String, "/system/control_status", TRANSIENT_QOS)
        self.create_subscription(ControlState, "/control/state", self._on_state, TRANSIENT_QOS)
        self.create_subscription(Twist, "/cmd_vel_manual", self._on_manual, 10)
        self.create_subscription(Twist, "/cmd_vel_nav", self._on_nav, 10)
        self.create_subscription(Twist, "/cmd_vel_sbus", self._on_sbus, 10)
        self.create_timer(self._publish_period_sec, self._publish_command)

    def _on_state(self, message: ControlState) -> None:
        self._state = message
        self._control_status_pub.publish(String(data=compatibility_control_status(
            emergency_stop=message.emergency_stop,
            requested_mode=message.requested_mode,
        )))

    def _on_manual(self, message: Twist) -> None:
        self._manual_command = message
        self._manual_received_ns = self.get_clock().now().nanoseconds

    def _on_nav(self, message: Twist) -> None:
        self._nav_command = message
        self._nav_received_ns = self.get_clock().now().nanoseconds

    def _on_sbus(self, message: Twist) -> None:
        self._sbus_command = message
        self._sbus_received_ns = self.get_clock().now().nanoseconds

    def _fresh(self, received_ns: int | None) -> bool:
        return received_ns is not None and (self.get_clock().now().nanoseconds - received_ns) <= int(self._command_timeout_sec * 1e9)

    def _publish_command(self) -> None:
        if self._state is None:
            self._command_pub.publish(Twist())
            return
        source = select_source(
            emergency_stop=self._state.emergency_stop,
            effective_source=self._state.effective_source,
            auto_permitted=self._state.auto_permitted,
            manual_command_fresh=self._fresh(self._manual_received_ns),
            nav_command_fresh=self._fresh(self._nav_received_ns),
            sbus_command_fresh=self._fresh(self._sbus_received_ns),
        )
        if source == Source.MANUAL:
            self._command_pub.publish(self._manual_command)
        elif source == Source.AUTO:
            self._command_pub.publish(self._nav_command)
        elif source == Source.SBUS:
            self._command_pub.publish(self._sbus_command)
        else:
            self._command_pub.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CommandArbiter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
