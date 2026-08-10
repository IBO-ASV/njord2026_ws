"""Allowlisted vessel-side executor for typed critical operator commands."""

from __future__ import annotations

import rclpy
import time
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from njord_interfaces.msg import MissionStatus, Nav2RuntimeStatus, OperatorCommand, OperatorResponse
from njord_interfaces.srv import GetMissionStatus, SetControlMode, StartTask, StopTask


_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                  durability=DurabilityPolicy.VOLATILE)
_STATUS_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)


class OperatorDispatcher(Node):
    def __init__(self):
        super().__init__("operator_dispatcher")
        self._status = MissionStatus()
        self._runtime = Nav2RuntimeStatus()
        self._control = None
        self._requests = {}
        self._response_pub = self.create_publisher(OperatorResponse, "/critical_link/operator_response", _QOS)
        self.create_subscription(OperatorCommand, "/critical_link/operator_command", self._command, _QOS)
        self.create_subscription(MissionStatus, "/mission/status", self._mission, _STATUS_QOS)
        self.create_subscription(Nav2RuntimeStatus, "/runtime/nav2/status", self._runtime_status, _STATUS_QOS)
        from njord_interfaces.msg import ControlState
        self.create_subscription(ControlState, "/control/state", self._control_state, _STATUS_QOS)
        self._mode = self.create_client(SetControlMode, "/control/set_mode")
        self._start = self.create_client(StartTask, "/mission/start_task")
        self._stop = self.create_client(StopTask, "/mission/stop_task")
        self._um982 = self.create_client(Trigger, "/sensor/vehicle_gnss/hot_restart")

    def _mission(self, message): self._status = message
    def _runtime_status(self, message): self._runtime = message
    def _control_state(self, message): self._control = message

    def _respond(self, command, result, message):
        response = OperatorResponse()
        response.request_id, response.result_code, response.message = command.request_id, result, message
        response.mission_state = self._status.state
        response.requested_mode = self._status.requested_control_mode
        response.effective_mode = self._status.effective_control_source
        response.task_id = self._status.task_id
        response.active_nav2_profile = self._runtime.active_profile
        response.runtime_state = self._runtime.state
        cached = self._requests.get(command.request_id)
        if cached and cached["fingerprint"] == self._fingerprint(command):
            cached["response"] = response
        self._response_pub.publish(response)

    @staticmethod
    def _fingerprint(command):
        return (command.command, command.target, command.requested_mode, command.task_id)

    def _admit(self, command):
        if command.request_id == 0:
            self._respond(command, OperatorResponse.REJECTED, "request_id must not be zero")
            return False
        now = time.monotonic()
        self._requests = {key: value for key, value in self._requests.items()
                          if value["expires"] > now}
        existing = self._requests.get(command.request_id)
        fingerprint = self._fingerprint(command)
        if existing:
            if existing["fingerprint"] != fingerprint:
                self._respond(command, OperatorResponse.REJECTED, "request_id payload mismatch")
            elif existing["response"] is not None:
                self._response_pub.publish(existing["response"])
            return False
        if len(self._requests) >= 256:
            self._respond(command, OperatorResponse.UNAVAILABLE, "operator request cache is full")
            return False
        self._requests[command.request_id] = {
            "fingerprint": fingerprint, "response": None, "expires": now + 60.0}
        return True

    def _safe_manual_idle(self):
        return self._control and self._control.requested_mode == 0 and self._control.effective_source in {0, 1} and self._status.state == MissionStatus.STATE_IDLE

    def _call(self, command, client, request):
        if not client.service_is_ready():
            self._respond(command, OperatorResponse.UNAVAILABLE, f"{client.srv_name} is unavailable")
            return
        client.call_async(request).add_done_callback(lambda future: self._finish(command, future))

    def _finish(self, command, future):
        try:
            result = future.result()
            accepted = getattr(result, "accepted", getattr(result, "success", False))
            self._respond(command, OperatorResponse.OK if accepted else OperatorResponse.REJECTED, result.message)
        except Exception as error:
            self._respond(command, OperatorResponse.FAILED, str(error))

    def _command(self, command):
        if not self._admit(command):
            return
        if command.target != OperatorCommand.MINIPC:
            self._respond(command, OperatorResponse.UNAVAILABLE, "Jetson relay is unavailable")
        elif command.command == OperatorCommand.SET_MODE:
            if command.requested_mode not in {0, 1}:
                self._respond(command, OperatorResponse.REJECTED, "requested mode must be MANUAL or AUTO")
                return
            request = SetControlMode.Request(requested_mode=command.requested_mode, request_id=str(command.request_id))
            self._call(command, self._mode, request)
        elif command.command in {OperatorCommand.TASK_CHECK, OperatorCommand.TASK_START}:
            if not command.task_id:
                self._respond(command, OperatorResponse.REJECTED, "task_id is required")
                return
            request = StartTask.Request(task_id=command.task_id,
                request_auto_mode=command.command == OperatorCommand.TASK_START,
                dry_run=command.command == OperatorCommand.TASK_CHECK, request_id=str(command.request_id))
            self._call(command, self._start, request)
        elif command.command == OperatorCommand.TASK_STOP:
            if not self._status.execution_id:
                self._respond(command, OperatorResponse.REJECTED, "no active mission")
                return
            self._call(command, self._stop, StopTask.Request(execution_id=self._status.execution_id, return_to_manual=True))
        elif command.command == OperatorCommand.TASK_STATUS:
            self._respond(command, OperatorResponse.OK, self._status.message or "mission status")
        elif command.command == OperatorCommand.UM982_HOT_RESTART:
            if not self._safe_manual_idle():
                self._respond(command, OperatorResponse.REJECTED, "UM982 restart requires MANUAL, ZERO/MANUAL source, and idle mission")
                return
            self._call(command, self._um982, Trigger.Request())
        else:
            self._respond(command, OperatorResponse.REJECTED, "unsupported operator command")


def main():
    rclpy.init()
    rclpy.spin(OperatorDispatcher())
    rclpy.shutdown()
