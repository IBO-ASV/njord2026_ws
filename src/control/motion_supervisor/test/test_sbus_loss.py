"""SBUS loss zeros thrust without shortening the ownership grace period."""
from unittest.mock import Mock

import pytest
pytest.importorskip("rclpy")
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from control_interfaces.msg import ControlState
from motion_supervisor.command_arbiter_node import CommandArbiter


def test_sbus_loss_and_recovery():
    rclpy.init()
    node = CommandArbiter()
    try:
        node._command_pub = Mock()
        node._state = ControlState(effective_source=ControlState.SOURCE_SBUS)
        command = Twist()
        command.linear.x = 0.7
        node._on_sbus_joy(Joy(buttons=[0]))
        node._on_sbus(command)
        node._publish_command()
        assert node._command_pub.publish.call_args.args[0].linear.x == 0.7
        node._sbus_received_ns -= 600_000_000
        node._publish_command()
        assert node._command_pub.publish.call_args.args[0] == Twist()
        assert node._fresh(node._sbus_received_ns)  # Still owns the grace period.
        node._on_sbus(command)
        node._on_sbus_joy(Joy(buttons=[1]))  # Explicit failsafe: no timeout wait.
        node._publish_command()
        assert node._command_pub.publish.call_args.args[0] == Twist()
        node._on_sbus_joy(Joy(buttons=[0]))
        node._on_sbus(command)
        node._publish_command()
        assert node._command_pub.publish.call_args.args[0].linear.x == 0.7
        node._state.effective_source = ControlState.SOURCE_MANUAL
        manual = Twist()
        manual.linear.y = 0.2
        node._on_manual(manual)
        node._publish_command()
        assert node._command_pub.publish.call_args.args[0] == manual
    finally:
        node.destroy_node()
        rclpy.shutdown()
