#!/usr/bin/env python3
"""Distribute editable ROS 2 parameters from the ground PC to each machine."""

from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import String
import yaml


def role_parameters(document, role):
    """Return node-to-parameter mappings for one role, rejecting malformed input."""
    if not isinstance(document, dict) or not isinstance(document.get(role, {}), dict):
        raise ValueError(f"{role} must map node names to parameter mappings")
    targets = document.get(role, {})
    for node_name, parameters in targets.items():
        if not isinstance(node_name, str) or not isinstance(parameters, dict):
            raise ValueError("each role entry must be a node name and parameter mapping")
        if any(not isinstance(name, str) or not valid_parameter_value(value)
               for name, value in parameters.items()):
            raise ValueError("parameter names must be strings and values cannot be mappings")
    return targets


def valid_parameter_value(value):
    if type(value) in (bool, int, float, str):
        return True
    return bool(value) and isinstance(value, list) and all(
        type(item) is type(value[0]) for item in value
    ) and type(value[0]) in (bool, int, float, str)


class RuntimeParameterSync(Node):
    def __init__(self):
        super().__init__("runtime_parameter_sync")
        self.role = self.declare_parameter("role", "groundpc").value
        self.config_file = Path(self.declare_parameter("config_file", "").value)
        self.publish_updates = self.declare_parameter("publish_updates", False).value
        self.last_mtime_ns = None
        self.parameter_clients = []
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(String, "/runtime_parameter_updates", qos)
        self.create_subscription(String, "/runtime_parameter_updates", self.receive, qos)
        if self.publish_updates:
            self.create_timer(1.0, self.publish_if_changed)

    def publish_if_changed(self):
        try:
            mtime_ns = self.config_file.stat().st_mtime_ns
            if mtime_ns == self.last_mtime_ns:
                return
            document = yaml.safe_load(self.config_file.read_text()) or {}
            for role in ("groundpc", "minipc", "jetson"):
                role_parameters(document, role)
        except (OSError, ValueError, yaml.YAMLError) as error:
            self.get_logger().error(f"runtime parameter config not published: {error}")
            return
        self.last_mtime_ns = mtime_ns
        self.publisher.publish(String(data=yaml.safe_dump(document)))
        self.get_logger().info(f"published {self.config_file}")

    def receive(self, message):
        try:
            targets = role_parameters(yaml.safe_load(message.data) or {}, self.role)
        except (ValueError, yaml.YAMLError) as error:
            self.get_logger().error(f"runtime parameter update rejected: {error}")
            return
        for node_name, values in targets.items():
            client = self.create_client(SetParameters, f"{node_name}/set_parameters")
            if not client.service_is_ready():
                self.get_logger().warning(f"{node_name}: parameter service unavailable")
                continue
            request = SetParameters.Request()
            request.parameters = [
                Parameter(name, value=value).to_parameter_msg() for name, value in values.items()
            ]
            future = client.call_async(request)
            future.add_done_callback(lambda done, node=node_name: self.report(node, done))
            self.parameter_clients.append(client)

    def report(self, node_name, future):
        try:
            rejected = [result.reason for result in future.result().results if not result.successful]
            if rejected:
                self.get_logger().error(f"{node_name}: " + "; ".join(rejected))
            else:
                self.get_logger().info(f"{node_name}: runtime parameters applied")
        except Exception as error:  # Service failures must not stop future edits.
            self.get_logger().error(f"{node_name}: parameter update failed: {error}")


def main():
    rclpy.init()
    node = RuntimeParameterSync()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
