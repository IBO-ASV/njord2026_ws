"""Tuning commands establish freshness without bypassing AUTO permission."""
from unittest.mock import Mock

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from control_interfaces.msg import ControlState
from motion_supervisor.safety_supervisor_node import SafetySupervisor


def test_tuning_freshness_obeys_active_flag_and_auto_gate():
    rclpy.init()
    node = SafetySupervisor()
    try:
        node._state_pub = Mock()
        node._requested_mode = ControlState.MODE_AUTO
        node._emergency_stop = False
        node._decision = lambda: (True, ())
        node._on_tuning_active(Bool(data=True))
        node._on_tune_command(Twist())
        node._publish_state()
        state = node._state_pub.publish.call_args.args[0]
        assert state.effective_source == ControlState.SOURCE_AUTO
        assert state.nav_command_fresh
        node._decision = lambda: (False, ())
        node._publish_state()
        assert node._state_pub.publish.call_args.args[0].effective_source == ControlState.SOURCE_ZERO
        node._decision = lambda: (True, ())
        node._on_tuning_active(Bool(data=False))
        node._publish_state()
        assert not node._state_pub.publish.call_args.args[0].nav_command_fresh
        node._on_tuning_active(Bool(data=True))
        node._tuning_received_ns -= int((node._policy.nav_command_timeout_sec + 1) * 1e9)
        node._publish_state()
        assert not node._state_pub.publish.call_args.args[0].nav_command_fresh
    finally:
        node.destroy_node()
        rclpy.shutdown()
