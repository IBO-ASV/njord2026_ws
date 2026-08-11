"""Own the one replaceable Nav2 runtime process on the miniPC."""

from __future__ import annotations

import os
import signal
import subprocess
import threading

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from njord_interfaces.action import ConfigureSystem
from njord_interfaces.msg import Nav2RuntimeStatus


_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                  durability=DurabilityPolicy.TRANSIENT_LOCAL)


class RuntimeManager(Node):
    def __init__(self):
        super().__init__("runtime_manager")
        self.declare_parameter("sigint_timeout_sec", 10.0)
        self.declare_parameter("sigterm_timeout_sec", 5.0)
        self._process = None
        self._profile = ""
        self._lock = threading.Lock()
        self._pub = self.create_publisher(Nav2RuntimeStatus, "/runtime/nav2/status", _QOS)
        self._monitor_timer = self.create_timer(0.2, self._monitor_process)
        self._action = ActionServer(
            self,
            ConfigureSystem,
            "/system/configure",
            self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
        )
        self._publish(Nav2RuntimeStatus.STOPPED, "")

    def _monitor_process(self):
        """Turn an exited launch process into an observable mission failure."""
        with self._lock:
            if self._process is None:
                return
            exit_code = self._process.poll()
            if exit_code is None:
                return
            self._process = None
            self._publish(Nav2RuntimeStatus.FAILED, f"Nav2 runtime exited with code {exit_code}")

    def _goal(self, goal):
        return GoalResponse.ACCEPT if goal.parameter == "nav2_profile" and goal.value in {"task1", "task2", "task3"} else GoalResponse.REJECT

    @staticmethod
    def _cancel(_goal):
        return CancelResponse.ACCEPT

    def _publish(self, state, message):
        message_out = Nav2RuntimeStatus()
        message_out.stamp = self.get_clock().now().to_msg()
        message_out.state, message_out.active_profile, message_out.message = state, self._profile, message
        self._pub.publish(message_out)

    def _stop(self):
        process = self._process
        if not process or process.poll() is not None:
            self._process = None
            return
        self._publish(Nav2RuntimeStatus.STOPPING, "stopping previous Nav2 runtime")
        for sig, timeout in ((signal.SIGINT, self.get_parameter("sigint_timeout_sec").value),
                             (signal.SIGTERM, self.get_parameter("sigterm_timeout_sec").value),
                             (signal.SIGKILL, 0)):
            os.killpg(process.pid, sig)
            try:
                process.wait(timeout=timeout or None)
                break
            except subprocess.TimeoutExpired:
                continue
        self._process = None

    def _execute(self, handle):
        profile = handle.request.value
        with self._lock:
            try:
                if self._profile != profile:
                    self._stop()
                    self._profile = profile
                    self._publish(Nav2RuntimeStatus.STARTING, "starting Nav2 runtime")
                    self._process = subprocess.Popen(
                        ["ros2", "launch", "robot", "task_runtime.launch.py", f"profile:={profile}"],
                        start_new_session=True)
                self._publish(Nav2RuntimeStatus.READY, "Nav2 runtime launch started")
                result = ConfigureSystem.Result(success=True, message=f"active Nav2 profile: {profile}")
                handle.succeed()
                return result
            except Exception as error:
                self._publish(Nav2RuntimeStatus.FAILED, str(error))
                handle.abort()
                return ConfigureSystem.Result(success=False, message=str(error))


def main():
    rclpy.init()
    rclpy.spin(RuntimeManager())
    rclpy.shutdown()
