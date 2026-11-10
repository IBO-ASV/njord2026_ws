"""Operator CLI which only calls the canonical Mission Manager APIs."""

from __future__ import annotations

import argparse
import sys
import time
import uuid

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from njord_interfaces.action import RunTask
from njord_interfaces.srv import GetMissionStatus, ListTasks, SetControlMode, StartTask, StopTask


class TaskClient(Node):
    def __init__(self) -> None:
        super().__init__("njord_task")
        self.list_client = self.create_client(ListTasks, "/mission/list_tasks")
        self.status_client = self.create_client(GetMissionStatus, "/mission/get_status")
        self.start_client = self.create_client(StartTask, "/mission/start_task")
        self.stop_client = self.create_client(StopTask, "/mission/stop_task")
        self.set_mode_client = self.create_client(SetControlMode, "/control/set_mode")
        self.run_client = ActionClient(self, RunTask, "/mission/run_task")

    def call(self, client, request, timeout: float = 5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"service {client.srv_name} is unavailable")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError(f"timed out waiting for {client.srv_name}")
        return future.result()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="njord-task", description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("list")
    commands.add_parser("status")
    check = commands.add_parser("check")
    check.add_argument("task_id")
    start = commands.add_parser("start")
    start.add_argument("task_id")
    start.add_argument("--auto", action="store_true", help="request AUTO; safety policy still decides")
    start.add_argument("--dry-run", action="store_true")
    stop = commands.add_parser("stop")
    stop.add_argument("--execution-id", default="", help="defaults to the current mission execution")
    commands.add_parser("manual", help="request MANUAL through the canonical control manager")
    return parser


def main(argv=None) -> int:
    args = _parser().parse_args(argv)
    rclpy.init(args=None)
    client = TaskClient()
    try:
        if args.command == "list":
            response = client.call(client.list_client, ListTasks.Request())
            for task in response.tasks:
                print(f"{task.task_id}\t{task.display_name}\tavailability={task.availability}\t{task.reason}")
            return 0
        if args.command == "status":
            status = client.call(client.status_client, GetMissionStatus.Request()).status
            print(f"state={status.state} task={status.task_id or '-'} execution={status.execution_id or '-'}")
            print(f"stage={status.stage or '-'} progress={status.progress:.0%} auto_permitted={status.auto_permitted}")
            if status.message:
                print(status.message)
            for reason in status.inhibit_reasons:
                print(f"inhibit: {reason}")
            return 0
        if args.command == "check":
            return _check(client, args.task_id)
        if args.command == "start":
            request = StartTask.Request()
            request.task_id = args.task_id
            request.request_auto_mode = args.auto
            request.dry_run = args.dry_run
            request.request_id = f"njord-task-{uuid.uuid4()}"
            response = client.call(client.start_client, request)
            print(response.message)
            if response.execution_id:
                print(f"execution_id={response.execution_id}")
            return 0 if response.accepted else 2
        if args.command == "stop":
            execution_id = args.execution_id
            if not execution_id:
                execution_id = client.call(client.status_client, GetMissionStatus.Request()).status.execution_id
            if not execution_id:
                print("no active execution", file=sys.stderr)
                return 2
            request = StopTask.Request()
            request.execution_id = execution_id
            request.return_to_manual = True
            response = client.call(client.stop_client, request)
            print(response.message)
            return 0 if response.accepted else 2
        if args.command == "manual":
            request = SetControlMode.Request()
            request.requested_mode = SetControlMode.Request.MODE_MANUAL
            request.request_id = f"njord-task-manual-{uuid.uuid4()}"
            response = client.call(client.set_mode_client, request)
            print(response.message)
            return 0 if response.accepted else 2
    except RuntimeError as exc:
        print(f"njord-task: {exc}", file=sys.stderr)
        return 3
    finally:
        client.destroy_node()
        rclpy.shutdown()
    return 1


def _check(client: TaskClient, task_id: str) -> int:
    if not client.run_client.wait_for_server(timeout_sec=5.0):
        print("njord-task: /mission/run_task is unavailable", file=sys.stderr)
        return 3
    goal = RunTask.Goal()
    goal.task_id = task_id
    goal.request_auto_mode = False
    goal.dry_run = True
    goal.request_id = f"njord-task-check-{uuid.uuid4()}"
    future = client.run_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(client, future, timeout_sec=5.0)
    if not future.done() or not future.result().accepted:
        print("check rejected", file=sys.stderr)
        return 2
    result_future = future.result().get_result_async()
    rclpy.spin_until_future_complete(client, result_future, timeout_sec=5.0)
    if not result_future.done():
        print("check timed out", file=sys.stderr)
        return 3
    result = result_future.result().result
    print(result.message)
    return 0 if result.result_code == RunTask.Result.SUCCEEDED else 2
