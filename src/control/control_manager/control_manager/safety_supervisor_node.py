"""Single owner of AUTO permission and canonical ControlState."""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String, UInt8

from njord_interfaces.msg import ControlState, HealthSignal, HealthState
from njord_interfaces.srv import SetControlMode

from .arbitration import derive_control_state
from .policy import PolicyError, SafetyInputs, evaluate_auto_permission, load_policy


TRANSIENT_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class SafetySupervisor(Node):
    """Computes permission only; command forwarding belongs to CommandArbiter."""

    def __init__(self) -> None:
        super().__init__("safety_supervisor")
        default_policy = str(Path(get_package_share_directory("control_manager")) / "config" / "control_policy.yaml")
        self._policy_file = Path(self.declare_parameter("policy_file", default_policy).value)
        self._publish_period_sec = float(self.declare_parameter("publish_period_sec", 0.05).value)
        if self._publish_period_sec <= 0:
            raise ValueError("publish_period_sec must be positive")
        try:
            self._policy = load_policy(self._policy_file)
        except PolicyError as exc:
            self.get_logger().fatal("Unsafe control policy: %s", exc)
            raise RuntimeError(f"unsafe control policy: {exc}") from exc

        self._requested_mode = SetControlMode.Request.MODE_MANUAL
        self._emergency_stop = True
        self._nav2_ready = False
        self._task_ready = False
        self._task_requirements_ready = False
        self._active_policy = ""
        self._health_states: dict[str, int] = {}
        self._health_summary_critical = False
        self._manual_received_ns: int | None = None
        self._nav_received_ns: int | None = None
        self._sbus_received_ns: int | None = None

        self._state_pub = self.create_publisher(ControlState, "/control/state", TRANSIENT_QOS)
        self.create_subscription(UInt8, "/control/requested_mode", self._on_requested_mode, TRANSIENT_QOS)
        self.create_subscription(UInt8, "/safety/emergency_stop", self._on_emergency_stop, 10)
        self.create_subscription(Bool, "/autonomy/ready", self._on_nav2_ready, 10)
        self.create_subscription(Bool, "/mission/task_ready", self._on_task_ready, TRANSIENT_QOS)
        self.create_subscription(Bool, "/mission/task_requirements_ready", self._on_task_requirements_ready, TRANSIENT_QOS)
        self.create_subscription(String, "/mission/active_control_policy", self._on_active_policy, TRANSIENT_QOS)
        from geometry_msgs.msg import Twist  # Imported here to keep policy tests ROS-free.
        self.create_subscription(Twist, "/cmd_vel_manual", self._on_manual_command, 10)
        self.create_subscription(Twist, "/cmd_vel_nav", self._on_nav_command, 10)
        self.create_subscription(Twist, "/cmd_vel_sbus", self._on_sbus_command, 10)
        self.create_subscription(HealthState, "/health/state", self._on_health_state, TRANSIENT_QOS)
        self.create_timer(self._publish_period_sec, self._publish_state)
        self._publish_state()

    def _on_requested_mode(self, message: UInt8) -> None:
        if message.data in {ControlState.MODE_MANUAL, ControlState.MODE_AUTO}:
            self._requested_mode = message.data

    def _on_emergency_stop(self, message: UInt8) -> None:
        self._emergency_stop = message.data != 0

    def _on_nav2_ready(self, message: Bool) -> None:
        self._nav2_ready = message.data

    def _on_task_ready(self, message: Bool) -> None:
        self._task_ready = message.data

    def _on_task_requirements_ready(self, message: Bool) -> None:
        self._task_requirements_ready = message.data

    def _on_active_policy(self, message: String) -> None:
        self._active_policy = message.data

    def _on_manual_command(self, _message) -> None:
        self._manual_received_ns = self.get_clock().now().nanoseconds

    def _on_nav_command(self, _message) -> None:
        self._nav_received_ns = self.get_clock().now().nanoseconds

    def _on_sbus_command(self, _message) -> None:
        self._sbus_received_ns = self.get_clock().now().nanoseconds

    def _on_health_state(self, message: HealthState) -> None:
        self._health_states = {signal.name: signal.state for signal in message.signals}
        self._health_summary_critical = message.summary_state == HealthState.ERROR

    def _fresh(self, received_ns: int | None, timeout_sec: float) -> bool:
        return received_ns is not None and (self.get_clock().now().nanoseconds - received_ns) <= int(timeout_sec * 1e9)

    def _decision(self):
        if not self._active_policy:
            return False, ((ControlState.INHIBIT_TASK_NOT_READY, "no active task control policy"),)
        try:
            requirements = self._policy.requirements_for(self._active_policy)
        except PolicyError as exc:
            return False, ((ControlState.INHIBIT_CONFIGURATION_ERROR, str(exc)),)
        decision = evaluate_auto_permission(
            requirements,
            SafetyInputs(
                emergency_stop=self._emergency_stop,
                nav2_ready=self._nav2_ready,
                task_ready=self._task_ready,
                health_states=self._health_states,
                health_summary_critical=self._health_summary_critical,
                task_requirements_ready=self._task_requirements_ready,
            ),
            health_ok_state=HealthSignal.OK,
            health_disabled_state=HealthSignal.DISABLED,
            inhibit_codes={
                "emergency_stop": ControlState.INHIBIT_EMERGENCY_STOP,
                "nav2_not_ready": ControlState.INHIBIT_NAV2_NOT_READY,
                "task_not_ready": ControlState.INHIBIT_TASK_NOT_READY,
                "require_ground_station": ControlState.INHIBIT_GROUND_STATION_UNAVAILABLE,
                "require_driver_heartbeat": ControlState.INHIBIT_DRIVER_HEARTBEAT,
                "require_localization_heartbeat": ControlState.INHIBIT_LOCALIZATION_HEARTBEAT,
                "require_rtk_fix": ControlState.INHIBIT_RTK_FIX,
                "critical_health": ControlState.INHIBIT_CRITICAL_HEALTH_FAULT,
                "task_requirement": ControlState.INHIBIT_TASK_REQUIREMENT,
            },
        )
        return decision.auto_permitted, decision.reasons

    def _publish_state(self) -> None:
        nav_fresh = self._fresh(self._nav_received_ns, self._policy.nav_command_timeout_sec)
        manual_fresh = self._fresh(self._manual_received_ns, self._policy.nav_command_timeout_sec)
        sbus_fresh = self._fresh(self._sbus_received_ns, self._policy.nav_command_timeout_sec)
        auto_permitted, reasons = self._decision()
        state = ControlState()
        state.stamp = self.get_clock().now().to_msg()
        state.requested_mode = self._requested_mode
        state.emergency_stop = self._emergency_stop
        state.auto_permitted = auto_permitted
        state.manual_command_fresh = manual_fresh
        state.nav_command_fresh = nav_fresh
        state.sbus_command_fresh = sbus_fresh
        state.inhibit_reason_codes = [code for code, _ in reasons]
        state.inhibit_reasons = [message for _, message in reasons]
        control_state, effective_source = derive_control_state(
            emergency_stop=self._emergency_stop,
            auto_requested=self._requested_mode == ControlState.MODE_AUTO,
            auto_permitted=auto_permitted,
            manual_command_fresh=manual_fresh,
            nav_command_fresh=nav_fresh,
            sbus_command_fresh=sbus_fresh,
        )
        state.state = int(control_state)
        state.effective_source = int(effective_source)
        self._state_pub.publish(state)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetySupervisor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
