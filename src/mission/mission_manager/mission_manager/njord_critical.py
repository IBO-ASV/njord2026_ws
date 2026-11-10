"""Send authenticated-transport operator commands through the one typed ROS topic."""

from __future__ import annotations

import argparse
import secrets
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from njord_interfaces.msg import OperatorCommand, OperatorResponse


_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                  durability=DurabilityPolicy.VOLATILE)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="njord-critical", description=__doc__)
    parser.add_argument("--topic", default="/critical_link/input/operator_command")
    commands = parser.add_subparsers(dest="command", required=True)
    mode = commands.add_parser("mode")
    mode.add_argument("value", choices=("manual", "auto"))
    task = commands.add_parser("task")
    task.add_argument("operation", choices=("check", "start", "stop", "status"))
    task.add_argument("task_id", nargs="?", default="")
    um982 = commands.add_parser("um982")
    um982.add_argument("operation", choices=("hot-restart",))
    bringup = commands.add_parser("bringup")
    bringup.add_argument("operation", choices=("restart",))
    bringup.add_argument("target", choices=("minipc", "jetson"))
    return parser


def make_command(args: argparse.Namespace, request_id: int) -> OperatorCommand:
    """Translate the intentionally small CLI grammar to the shared command contract."""
    message = OperatorCommand()
    message.request_id, message.target = request_id, OperatorCommand.MINIPC
    if args.command == "mode":
        message.command = OperatorCommand.SET_MODE
        message.requested_mode = (1 if args.value == "auto" else 0)
    elif args.command == "task":
        message.command = {
            "check": OperatorCommand.TASK_CHECK,
            "start": OperatorCommand.TASK_START,
            "stop": OperatorCommand.TASK_STOP,
            "status": OperatorCommand.TASK_STATUS,
        }[args.operation]
        message.task_id = args.task_id
        if args.operation in {"check", "start"} and not message.task_id:
            raise ValueError(f"task {args.operation} requires TASK_ID")
        if args.operation == "start":
            message.requested_mode = 1  # Task start always asks AUTO; safety remains authoritative.
    elif args.command == "um982":
        message.command = OperatorCommand.UM982_HOT_RESTART
    else:
        message.command = OperatorCommand.RESTART_BRINGUP
        message.target = OperatorCommand.JETSON if args.target == "jetson" else OperatorCommand.MINIPC
    return message


class CriticalClient(Node):
    def __init__(self, topic: str, request_id: int) -> None:
        super().__init__("njord_critical")
        self.response = None
        self.request_id = request_id
        self.publisher = self.create_publisher(OperatorCommand, topic, _QOS)
        self.create_subscription(OperatorResponse, "/critical_link/output/operator_response",
                                 self._response, _QOS)

    def _response(self, response: OperatorResponse) -> None:
        if response.request_id == self.request_id:
            self.response = response


def main(argv=None) -> int:
    args = _parser().parse_args(argv)
    request_id = secrets.randbits(64)
    try:
        command = make_command(args, request_id)
    except ValueError as error:
        print(f"njord-critical: {error}", file=sys.stderr)
        return 2
    rclpy.init(args=None)
    node = CriticalClient(args.topic, request_id)
    try:
        # Do not finish the short command burst before the local sender discovers us.
        deadline = time.monotonic() + 2.0
        while node.publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        # Publishing several times is safe because request_id deduplication is mandatory.
        # The critical-link sender polls network responses on a timer, so keep
        # spinning after the burst rather than treating its 0.75 s duration as
        # the response timeout.
        # A TASK_CHECK completes the full dry-run action (including serialized
        # map-coordinate projection) before the operator is told it is safe
        # to issue TASK_START.  Other commands remain short request/response
        # transactions.
        response_timeout = 15.0 if (
            args.command == "task" and args.operation == "check"
        ) else 2.0
        response_deadline = time.monotonic() + response_timeout
        for _ in range(3):
            node.publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=0.25)
            if node.response:
                break
        while not node.response and time.monotonic() < response_deadline:
            rclpy.spin_once(node, timeout_sec=min(0.1, response_deadline - time.monotonic()))
        if not node.response:
            print("njord-critical: no authenticated operator response", file=sys.stderr)
            return 3
        response = node.response
        print(response.message)
        print(f"mission_state={response.mission_state} profile={response.active_nav2_profile or '-'} "
              f"runtime_state={response.runtime_state}")
        return 0 if response.result_code == OperatorResponse.OK else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()
