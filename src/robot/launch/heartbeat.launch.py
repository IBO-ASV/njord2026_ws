"""Start the validated, role-owned topic heartbeat and health graph.

The role inventory is the source of truth for observation.  Safety policy is
deliberately kept in control_manager; this launch file only turns configured
topics into named HealthSignal messages and aggregates them into HealthState.
"""

from collections.abc import Mapping
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_ROLES = ("minipc", "jetson", "groundpc")


def _load_inventory(role):
    config_path = os.path.join(
        get_package_share_directory("diagnostic_monitors"),
        "config",
        "heartbeat",
        f"{role}.yaml",
    )
    try:
        with open(config_path, encoding="utf-8") as config_file:
            config = yaml.safe_load(config_file)
    except (OSError, yaml.YAMLError) as error:
        raise RuntimeError(f"Cannot load heartbeat inventory {config_path}: {error}") from error

    if not isinstance(config, Mapping) or config.get("role") != role:
        raise RuntimeError(f"{config_path} must declare role={role!r}")
    signals = config.get("signals")
    if not isinstance(signals, Mapping) or not signals:
        raise RuntimeError(f"{config_path} must contain a non-empty 'signals' mapping")

    validated = []
    for name, signal in signals.items():
        if not isinstance(name, str) or not name.strip() or not isinstance(signal, Mapping):
            raise RuntimeError(f"Invalid heartbeat signal in {config_path}: {name!r}")
        required = ("topic", "type", "timeout_sec", "expected_frequency_hz")
        missing = [key for key in required if key not in signal]
        if missing:
            raise RuntimeError(f"Signal {name!r} in {config_path} misses {missing}")
        topic = signal["topic"]
        topic_type = signal["type"]
        if not isinstance(topic, str) or not topic.startswith("/"):
            raise RuntimeError(f"Signal {name!r} topic must be an absolute ROS topic")
        if not isinstance(topic_type, str) or "/" not in topic_type:
            raise RuntimeError(f"Signal {name!r} type must look like package/msg/Type")
        try:
            timeout = float(signal["timeout_sec"])
            expected = float(signal["expected_frequency_hz"])
            minimum = float(signal.get("minimum_frequency_hz", expected))
        except (TypeError, ValueError) as error:
            raise RuntimeError(f"Signal {name!r} has invalid numeric values") from error
        if timeout <= 0.0 or minimum < 0.0 or minimum > expected:
            raise RuntimeError(f"Signal {name!r} has invalid timeout/frequency")
        enabled = signal.get("enabled", True)
        if not isinstance(enabled, bool):
            raise RuntimeError(f"Signal {name!r} enabled must be boolean")
        if enabled:
            validated.append((name, topic, topic_type, timeout, expected, minimum))
    aggregates = config.get("aggregates", {})
    if not isinstance(aggregates, Mapping):
        raise RuntimeError(f"{config_path} aggregates must be a mapping")
    signal_by_name = {name: (topic, topic_type) for name, topic, topic_type, _, _, _ in validated}
    validated_aggregates = []
    output_topics = set()
    for name, aggregate in aggregates.items():
        if not isinstance(name, str) or not name.strip() or not isinstance(aggregate, Mapping):
            raise RuntimeError(f"Invalid heartbeat aggregate in {config_path}: {name!r}")
        required = ("output_topic", "input_signals", "input_timeout_sec", "publish_period_sec")
        missing = [key for key in required if key not in aggregate]
        if missing:
            raise RuntimeError(f"Heartbeat aggregate {name!r} in {config_path} misses {missing}")
        output_topic = aggregate["output_topic"]
        input_names = aggregate["input_signals"]
        if not isinstance(output_topic, str) or not output_topic.startswith("/"):
            raise RuntimeError(f"Heartbeat aggregate {name!r} output_topic must be absolute")
        if output_topic in output_topics:
            raise RuntimeError(f"Duplicate heartbeat aggregate output_topic {output_topic!r}")
        if not isinstance(input_names, list) or not input_names or not all(isinstance(item, str) for item in input_names):
            raise RuntimeError(f"Heartbeat aggregate {name!r} input_signals must be a non-empty string list")
        unknown = [item for item in input_names if item not in signal_by_name]
        if unknown:
            raise RuntimeError(f"Heartbeat aggregate {name!r} references unknown signals {unknown}")
        try:
            timeout = float(aggregate["input_timeout_sec"])
            period = float(aggregate["publish_period_sec"])
        except (TypeError, ValueError) as error:
            raise RuntimeError(f"Heartbeat aggregate {name!r} has invalid timeout/period") from error
        if timeout <= 0.0 or period <= 0.0:
            raise RuntimeError(f"Heartbeat aggregate {name!r} timeout/period must be positive")
        output_topics.add(output_topic)
        validated_aggregates.append((
            name, output_topic,
            [signal_by_name[item][0] for item in input_names],
            [signal_by_name[item][1] for item in input_names],
            timeout, period,
        ))
    return validated, validated_aggregates


def _monitor(role, name, topic, topic_type, timeout, expected, minimum):
    # Keep signal names role-qualified on the transport so a Jetson/ground-PC
    # graph cannot accidentally overwrite a miniPC signal with the same name.
    return Node(
        package="diagnostic_monitors",
        executable="topic_heartbeat_monitor_node",
        name=f"heartbeat_{role}_{name}",
        output="screen",
        parameters=[
            {
                "monitor_name": name,
                "topic": topic,
                "topic_type": topic_type,
                "mode": "required_frequency",
                "expected_frequency": expected,
                "minimum_frequency": minimum,
                "timeout": timeout,
                "stale_timeout": max(timeout * 3.0, timeout + 1.0),
                "health_signal_topic": f"/health/signals/{role}",
            }
        ],
    )


def _aggregate(role, name, output_topic, input_topics, input_types, timeout, period):
    return Node(
        package="diagnostic_monitors",
        executable="heartbeat_aggregator_node",
        name=f"heartbeat_{role}_{name}",
        output="screen",
        parameters=[
            {
                "input_topics": input_topics,
                "input_types": input_types,
                "output_topic": output_topic,
                "input_timeout_sec": timeout,
                "publish_period_sec": period,
            }
        ],
    )


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    role = LaunchConfiguration("role").perform(context).strip().lower()
    if role not in _ROLES:
        raise RuntimeError(f"Unknown heartbeat role: {role!r}")

    signals, aggregates = _load_inventory(role)
    # These arguments remain accepted for compatibility with existing Jetson
    # bringup files.  The inventory still records all signals; disabling a
    # legacy branch only suppresses its monitor process for that launch.
    monitor_zed2i = LaunchConfiguration("heartbeat_monitor_zed2i").perform(context)
    monitor_lidar = LaunchConfiguration("heartbeat_monitor_lidar").perform(context)
    disabled = set()
    if monitor_zed2i.strip().lower() in {"0", "false", "no", "off"}:
        disabled.add("front_camera")
    if monitor_lidar.strip().lower() in {"0", "false", "no", "off"}:
        disabled.update({"lidar_points", "lidar_imu"})

    nodes = [
        _monitor(role, name, topic, topic_type, timeout, expected, minimum)
        for name, topic, topic_type, timeout, expected, minimum in signals
        if name not in disabled
    ]
    nodes.extend(
        _aggregate(role, name, output_topic, input_topics, input_types, timeout, period)
        for name, output_topic, input_topics, input_types, timeout, period in aggregates
    )
    # miniPC is the canonical vessel health owner.  Role-local summaries on
    # Jetson and ground-PC remain observable without creating competing
    # publishers of /health/state.
    health_topic = "/health/state" if role == "minipc" else f"/health/state/{role}"
    nodes.append(
        Node(
            package="diagnostic_monitors",
            executable="health_state_aggregator_node",
            name=f"health_state_aggregator_{role}",
            output="screen",
            parameters=[
                {
                    "input_topic": f"/health/signals/{role}",
                    "output_topic": health_topic,
                    "signal_timeout_sec": 10.0,
                    "publish_period_sec": 1.0,
                }
            ],
        )
    )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("role", choices=list(_ROLES), default_value="minipc"),
            DeclareLaunchArgument("heartbeat_monitor_zed2i", default_value="true"),
            DeclareLaunchArgument("heartbeat_monitor_lidar", default_value="true"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
