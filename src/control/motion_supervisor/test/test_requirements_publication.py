"""The GUI receives the same active requirements that inhibit AUTO."""

import json
import time
from pathlib import Path

import pytest
import yaml

pytest.importorskip("rclpy")
import rclpy
from motion_supervisor.safety_supervisor_node import TRANSIENT_QOS, SafetySupervisor
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from health_interfaces.msg import HealthSignal, HealthState


def test_required_stale_inputs_are_published_with_all_inhibit_reasons(tmp_path):
    policy = yaml.safe_load(
        (Path(__file__).parents[1] / "config/control_policy.yaml").read_text()
    )
    body = policy["control_policy"]
    body["health_signals"].append("driver_heartbeat")
    for key in (
        "require_driver_heartbeat",
        "require_localization_heartbeat",
        "require_rtk_fix",
    ):
        body["common"][key] = True
    filename = tmp_path / "policy.yaml"
    filename.write_text(yaml.safe_dump(policy))
    rclpy.init(args=["--ros-args", "-p", f"policy_file:={filename}"])
    supervisor = SafetySupervisor()
    observer = Node("requirements_test_observer")
    received = []
    observer.create_subscription(
        String,
        "/control/requirements",
        lambda m: received.append(json.loads(m.data)),
        TRANSIENT_QOS,
    )
    supervisor._active_policy = "task1"
    supervisor._emergency_stop = False
    supervisor._on_health_state(
        HealthState(
            signals=[
                HealthSignal(name="local_odometry", state=HealthSignal.STALE),
                HealthSignal(name="gnss", state=HealthSignal.UNKNOWN),
            ]
        )
    )
    executor = SingleThreadedExecutor()
    executor.add_node(supervisor)
    executor.add_node(observer)
    try:
        deadline = time.monotonic() + 3
        while (
            not any(r["policy"] == "task1" for r in received)
            and time.monotonic() < deadline
        ):
            executor.spin_once(timeout_sec=0.05)
        message = next(r for r in received if r["policy"] == "task1")
        for key in (
            "require_driver_heartbeat",
            "require_localization_heartbeat",
            "require_rtk_fix",
        ):
            assert message["requirements"][key]
        assert not message["preparation_permitted"]
        assert all(any(signal in reason for reason in message["preparation_inhibit_reasons"])
                   for signal in ("local_odometry", "gnss", "driver_heartbeat"))
        permitted, reasons = supervisor._decision()
        assert not permitted
        text = " ".join(reason for _, reason in reasons)
        for signal in ("local_odometry", "gnss", "driver_heartbeat"):
            assert signal in text
        # IDLE hides task permission behind "no active policy", but preparation
        # must still independently check health, even before Nav2/Task are ready.
        supervisor._active_policy = ""
        supervisor._health_states = {
            name: HealthSignal.OK for name in ("local_odometry", "gnss", "driver_heartbeat")
        }
        assert not supervisor._decision()[0]
        received.clear()
        deadline = time.monotonic() + 3
        while not any(r["policy"] == "" and r["preparation_permitted"] for r in received) and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        prepared = next(r for r in received if r["policy"] == "" and r["preparation_permitted"])
        assert not prepared["task_ready"]
        assert not prepared["nav2_ready"]
        supervisor._emergency_stop = True
        received.clear()
        deadline = time.monotonic() + 3
        while not any(not r["preparation_permitted"] for r in received) and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        blocked = next(r for r in received if not r["preparation_permitted"])
        assert "emergency stop is active" in blocked["preparation_inhibit_reasons"]
    finally:
        executor.shutdown()
        supervisor.destroy_node()
        observer.destroy_node()
        rclpy.shutdown()
