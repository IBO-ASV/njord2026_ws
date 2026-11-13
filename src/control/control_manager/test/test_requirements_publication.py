"""The GUI receives the same active requirements that inhibit AUTO."""

import json
from pathlib import Path
import time
import pytest
import yaml

pytest.importorskip("rclpy")
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from njord_interfaces.msg import HealthSignal, HealthState
from control_manager.safety_supervisor_node import SafetySupervisor, TRANSIENT_QOS


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
        permitted, reasons = supervisor._decision()
        assert not permitted
        text = " ".join(reason for _, reason in reasons)
        for signal in ("local_odometry", "gnss", "driver_heartbeat"):
            assert signal in text
    finally:
        executor.shutdown()
        supervisor.destroy_node()
        observer.destroy_node()
        rclpy.shutdown()
