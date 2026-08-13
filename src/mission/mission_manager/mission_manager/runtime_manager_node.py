"""Own the one replaceable Nav2 runtime process on the miniPC."""

from __future__ import annotations

import os
import signal
import subprocess
import threading
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from njord_interfaces.action import ConfigureSystem
from njord_interfaces.msg import Nav2RuntimeStatus


_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                  durability=DurabilityPolicy.TRANSIENT_LOCAL)


class RuntimeManager(Node):
    def __init__(self):
        super().__init__("runtime_manager")
        self.declare_parameter("sigint_timeout_sec", 1.0)
        self.declare_parameter("sigterm_timeout_sec", 0.5)
        self.declare_parameter("nav2_ready_timeout_sec", 45.0)
        self.declare_parameter("nav2_ready_poll_sec", 0.5)
        self.declare_parameter("nav2_lifecycle_query_timeout_sec", 3.0)
        self.declare_parameter("runtime_mode", "nav2")
        self.declare_parameter("restart_on_configure", False)
        self._process = None
        self._profile = ""
        self._lock = threading.Lock()
        self._runtime_pgid_file = Path("/tmp/njord_nav2_runtime.pgid")
        self._cleanup_orphaned_runtime()
        self._pub = self.create_publisher(Nav2RuntimeStatus, "/runtime/nav2/status", _QOS)
        self._action = ActionServer(
            self,
            ConfigureSystem,
            "/system/configure",
            self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
        )
        self._publish(Nav2RuntimeStatus.STOPPED, "")

    def _goal(self, goal):
        mode = str(self.get_parameter("runtime_mode").value)
        profiles = {"task2"} if mode == "task2_sim" else {"task1", "task2", "task3"}
        return GoalResponse.ACCEPT if goal.parameter == "nav2_profile" and goal.value in profiles else GoalResponse.REJECT

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
        if not process:
            self._process = None
            return
        if rclpy.ok():
            self._publish(Nav2RuntimeStatus.STOPPING, "stopping previous Nav2 runtime")
        # The launch parent can exit before one of its lifecycle-managed
        # children.  The process group remains valid in that case, so do not
        # merely clear the record based on ``process.poll()``: terminate the
        # whole group and prevent a PID-1-reparented Nav2 tree from surviving.
        stopped = self._terminate_process_group(
            process.pid,
            sigint_timeout=float(self.get_parameter("sigint_timeout_sec").value),
            sigterm_timeout=float(self.get_parameter("sigterm_timeout_sec").value),
        )
        self._process = None
        if stopped:
            self._clear_runtime_pgid(process.pid)
        else:
            self.get_logger().error(
                f"Nav2 process group {process.pid} survived SIGKILL; retaining recovery record"
            )

    @staticmethod
    def _terminate_process_group(pgid: int, *, sigint_timeout: float, sigterm_timeout: float) -> bool:
        for sig, timeout in ((signal.SIGINT, sigint_timeout),
                             (signal.SIGTERM, sigterm_timeout),
                             (signal.SIGKILL, 0.5)):
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                return True
            deadline = time.monotonic() + timeout
            while timeout > 0.0 and time.monotonic() < deadline:
                try:
                    os.killpg(pgid, 0)
                except ProcessLookupError:
                    return True
                time.sleep(0.1)
        try:
            os.killpg(pgid, 0)
        except ProcessLookupError:
            return True
        return False

    def _clear_runtime_pgid(self, pgid: int) -> None:
        try:
            if self._runtime_pgid_file.read_text(encoding="utf-8").strip() == str(pgid):
                self._runtime_pgid_file.unlink()
        except FileNotFoundError:
            pass

    def _cleanup_orphaned_runtime(self) -> None:
        """Stop Nav2 children left behind by a previous RuntimeManager process."""
        try:
            pgid = int(self._runtime_pgid_file.read_text(encoding="utf-8").strip())
        except (FileNotFoundError, ValueError):
            return
        if pgid <= 1:
            self._clear_runtime_pgid(pgid)
            return
        try:
            os.killpg(pgid, 0)
        except ProcessLookupError:
            self._clear_runtime_pgid(pgid)
            return
        members = subprocess.run(
            ["ps", "-eo", "pgid=,args="], capture_output=True, text=True, check=False
        ).stdout.splitlines()
        nav2_member = any(
            line.lstrip().startswith(f"{pgid} ") and
            ("task_runtime.launch.py" in line or "task2_sim.launch.py" in line or "nav2_" in line)
            for line in members
        )
        if not nav2_member:
            self.get_logger().warning(
                f"discarding stale Nav2 runtime process-group record {pgid} with no Nav2 members"
            )
            self._clear_runtime_pgid(pgid)
            return
        self.get_logger().warning(f"stopping orphaned Nav2 runtime process group {pgid}")
        if self._terminate_process_group(pgid, sigint_timeout=10.0, sigterm_timeout=5.0):
            self._clear_runtime_pgid(pgid)
        else:
            self.get_logger().error(
                f"orphaned Nav2 process group {pgid} survived SIGKILL; retaining recovery record"
            )

    def _wait_for_lifecycle_nodes_active(self, node_names: tuple[str, ...], label: str) -> None:
        """Block until every named Nav2 lifecycle node is ACTIVE.

        Discovering an action endpoint is insufficient: Nav2 exposes endpoints
        before the owning lifecycle node is ACTIVE. Querying lifecycle through
        the ROS 2 CLI uses an independent DDS participant, so it remains safe
        while this action server's execute callback is waiting.
        """
        timeout = float(self.get_parameter("nav2_ready_timeout_sec").value)
        poll = float(self.get_parameter("nav2_ready_poll_sec").value)
        query_timeout = float(self.get_parameter("nav2_lifecycle_query_timeout_sec").value)
        if timeout <= 0.0 or poll <= 0.0 or query_timeout <= 0.0:
            raise ValueError("Nav2 lifecycle readiness timeouts and poll interval must be positive")

        deadline = time.monotonic() + timeout
        last_output = "lifecycle state is not available yet"
        while time.monotonic() < deadline:
            if self._process is None or self._process.poll() is not None:
                raise RuntimeError(f"Nav2 runtime exited before {label} became active")
            inactive = []
            outputs = []
            for node_name in node_names:
                try:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0.0:
                        inactive.extend(node_names)
                        outputs.append("lifecycle readiness deadline reached")
                        break
                    result = subprocess.run(
                        ["ros2", "lifecycle", "get", f"/{node_name}"],
                        capture_output=True,
                        text=True,
                        # Starting a ROS CLI participant and discovering a service
                        # regularly takes longer than the polling interval.
                        timeout=min(query_timeout, remaining),
                        check=False,
                    )
                    output = (result.stdout + result.stderr).strip()
                    if result.returncode != 0 or not output.lower().startswith("active"):
                        inactive.append(node_name)
                        outputs.append(f"/{node_name}: {output or 'not available'}")
                except subprocess.TimeoutExpired:
                    inactive.append(node_name)
                    outputs.append(f"/{node_name}: lifecycle query timed out")
            if not inactive:
                self.get_logger().info(f"{label} lifecycle state is active")
                return
            last_output = "; ".join(outputs)
            time.sleep(min(poll, max(0.0, deadline - time.monotonic())))

        raise RuntimeError(
            f"timed out after {timeout:g}s waiting for {label} to become active: {last_output}"
        )

    def _wait_for_service(self, service_name: str) -> None:
        """Wait for the child sim's localization service before route projection."""
        timeout = float(self.get_parameter("nav2_ready_timeout_sec").value)
        poll = float(self.get_parameter("nav2_ready_poll_sec").value)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._process is None or self._process.poll() is not None:
                raise RuntimeError(f"Nav2 runtime exited before {service_name} became available")
            result = subprocess.run(
                ["ros2", "service", "list"], capture_output=True, text=True, check=False
            )
            if result.returncode == 0 and service_name in result.stdout.splitlines():
                return
            time.sleep(min(poll, max(0.0, deadline - time.monotonic())))
        raise RuntimeError(f"timed out after {timeout:g}s waiting for {service_name}")

    def _wait_for_bt_navigator_active(self) -> None:
        """Block until Nav2 has activated the Task 1/3 action server owner."""
        self._wait_for_lifecycle_nodes_active(("bt_navigator",), "bt_navigator")

    def shutdown(self):
        """Stop the Nav2 process group before this manager exits.

        ``task_runtime.launch.py`` is intentionally started in its own process
        session so profile switches can terminate the complete Nav2 tree.  It
        must therefore be stopped explicitly when the enclosing miniPC
        bringup receives Ctrl-C; otherwise it is reparented to PID 1.
        """
        with self._lock:
            self._stop()
            self._profile = ""

    def _execute(self, handle):
        profile = handle.request.value
        with self._lock:
            try:
                mode = str(self.get_parameter("runtime_mode").value)
                if mode not in {"nav2", "task2_sim"}:
                    raise ValueError(f"unknown runtime_mode: {mode}")
                # A child launch can be stopped externally while this manager
                # remains alive.  Treat a dead or missing child exactly like
                # a profile change; otherwise a later task request would be
                # reported READY with no Nav2 action server.
                runtime_is_alive = self._process is not None and self._process.poll() is None
                if (self._profile != profile or not runtime_is_alive
                        or bool(self.get_parameter("restart_on_configure").value)):
                    self._stop()
                    self._profile = profile
                    self._publish(Nav2RuntimeStatus.STARTING, "starting Nav2 runtime")
                    command = (
                        ["ros2", "launch", "task2_sim", "task2_sim.launch.py", "mission_managed:=true"]
                        if mode == "task2_sim"
                        else ["ros2", "launch", "robot", "task_runtime.launch.py", f"profile:={profile}"]
                    )
                    self._process = subprocess.Popen(command, start_new_session=True)
                    self._runtime_pgid_file.write_text(f"{self._process.pid}\n", encoding="utf-8")
                    if profile in {"task1", "task3"}:
                        self._wait_for_bt_navigator_active()
                    elif mode == "task2_sim":
                        self._wait_for_service("/fromLL")
                    elif profile == "task2":
                        self._wait_for_lifecycle_nodes_active(
                            ("controller_server", "velocity_smoother"),
                            "Task 2 controller stack",
                        )
                self._publish(Nav2RuntimeStatus.READY, "Nav2 runtime started")
                result = ConfigureSystem.Result(success=True, message=f"active Nav2 profile: {profile}")
                handle.succeed()
                return result
            except Exception as error:
                self._stop()
                self._profile = ""
                self._publish(Nav2RuntimeStatus.FAILED, str(error))
                handle.abort()
                return ConfigureSystem.Result(success=False, message=str(error))


def main():
    rclpy.init()
    node = RuntimeManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
