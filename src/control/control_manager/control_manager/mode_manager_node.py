"""Owner of requested control mode and the typed SetControlMode API."""

from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String, UInt8

from njord_interfaces.srv import SetControlMode


TRANSIENT_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class ModeManager(Node):
    """Stores operator intent; it deliberately does not grant AUTO permission."""

    def __init__(self) -> None:
        super().__init__("mode_manager")
        self._requested_mode = SetControlMode.Request.MODE_MANUAL
        self._requested_mode_pub = self.create_publisher(UInt8, "/control/requested_mode", TRANSIENT_QOS)
        # Retain the old topic as an input/output adapter while legacy joystick code exists.
        self._legacy_mode_pub = self.create_publisher(String, "/system/operating_mode", TRANSIENT_QOS)
        self.create_subscription(String, "/system/operating_mode", self._on_legacy_mode, TRANSIENT_QOS)
        self.create_service(SetControlMode, "/control/set_mode", self._set_mode)
        self._publish_requested_mode()

    def _publish_requested_mode(self) -> None:
        self._requested_mode_pub.publish(UInt8(data=self._requested_mode))

    def _set_requested_mode(self, requested_mode: int, *, publish_legacy: bool) -> None:
        self._requested_mode = requested_mode
        self._publish_requested_mode()
        if publish_legacy:
            label = "auto" if requested_mode == SetControlMode.Request.MODE_AUTO else "manual"
            self._legacy_mode_pub.publish(String(data=label))

    def _on_legacy_mode(self, message: String) -> None:
        if message.data == "manual":
            self._set_requested_mode(SetControlMode.Request.MODE_MANUAL, publish_legacy=False)
        elif message.data == "auto":
            self._set_requested_mode(SetControlMode.Request.MODE_AUTO, publish_legacy=False)
        else:
            self.get_logger().warning("Ignoring invalid legacy operating mode %r", message.data)

    def _set_mode(self, request: SetControlMode.Request, response: SetControlMode.Response) -> SetControlMode.Response:
        if request.requested_mode not in {
            SetControlMode.Request.MODE_MANUAL,
            SetControlMode.Request.MODE_AUTO,
        }:
            response.accepted = False
            response.reason_code = SetControlMode.Response.INVALID_MODE
            response.message = "requested_mode must be MODE_MANUAL or MODE_AUTO"
            return response
        self._set_requested_mode(request.requested_mode, publish_legacy=True)
        response.accepted = True
        response.reason_code = SetControlMode.Response.ACCEPTED
        response.message = "requested AUTO remains subject to safety policy" if (
            request.requested_mode == SetControlMode.Request.MODE_AUTO
        ) else "MANUAL requested"
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ModeManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
