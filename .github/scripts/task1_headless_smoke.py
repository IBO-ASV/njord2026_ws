#!/usr/bin/env python3
"""Observe the non-hardware Task 1 command path in the simulator.

The test supplies only the two interfaces outside the simulator's source
closure: navsat_transform's /fromLL service and Nav2's
NavigateThroughPoses action.  It deliberately does not open a serial device,
start a physical driver, or test Nav2's planner implementation.  Its contract
is the Task 1 simulator's TF, waypoint-action, and command-routing boundary.
"""

import argparse
import time

from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateThroughPoses
import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from robot_localization.srv import FromLL
from std_msgs.msg import Float32MultiArray
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray


class Task1HeadlessSmoke(Node):
    """Mocks external navigation interfaces and observes sim-only topics."""

    def __init__(self):
        super().__init__("task1_headless_smoke")
        self.map_to_odom = False
        self.odom_to_base = False
        self.waypoint_markers = False
        self.valid_goal = False
        self.cmd_vel_forwarded = False
        self.nonzero_thruster_command = False
        self.invalid_goal_reason = ""

        self.create_service(FromLL, "/fromLL", self._from_ll_callback)
        self._action_server = ActionServer(
            self,
            NavigateThroughPoses,
            "/navigate_through_poses",
            execute_callback=self._execute_goal,
            goal_callback=lambda _goal: GoalResponse.ACCEPT,
        )
        self.nav_command_publisher = self.create_publisher(Twist, "/cmd_vel_nav", 10)

        self.create_subscription(TFMessage, "/tf", self._tf_callback, 100)
        tf_static_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TFMessage, "/tf_static", self._tf_callback, tf_static_qos)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_callback, 10)
        self.create_subscription(
            Float32MultiArray,
            "/thruster_command",
            self._thruster_callback,
            10,
        )
        marker_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            MarkerArray,
            "/sim/task1_waypoint_markers",
            self._marker_callback,
            marker_qos,
        )
        self.create_timer(0.1, self._publish_navigation_command)

    def _from_ll_callback(self, request, response):
        # The absolute coordinates are irrelevant to this integration smoke,
        # but the mapping must remain non-degenerate so the waypoint node
        # exercises its projection validation before submitting the action.
        response.map_point.x = (request.ll_point.longitude - 10.423) * 100000.0
        response.map_point.y = (request.ll_point.latitude - 63.440) * 100000.0
        response.map_point.z = request.ll_point.altitude
        return response

    def _execute_goal(self, goal_handle):
        poses = goal_handle.request.poses
        if len(poses) != 11:
            self.invalid_goal_reason = f"expected 11 Task1 poses, got {len(poses)}"
        elif any(pose.header.frame_id != "map" for pose in poses):
            frames = sorted({pose.header.frame_id for pose in poses})
            self.invalid_goal_reason = f"Task1 goal frames were {frames}, expected ['map']"
        else:
            self.valid_goal = True
        goal_handle.succeed()
        return NavigateThroughPoses.Result()

    def _tf_callback(self, message):
        for transform in message.transforms:
            parent = transform.header.frame_id
            child = transform.child_frame_id
            self.map_to_odom = self.map_to_odom or (parent == "map" and child == "odom")
            self.odom_to_base = self.odom_to_base or (parent == "odom" and child == "base_link")

    def _marker_callback(self, message):
        self.waypoint_markers = self.waypoint_markers or bool(message.markers)

    def _publish_navigation_command(self):
        if not self.valid_goal:
            return
        message = Twist()
        message.linear.x = 0.10
        message.angular.z = 0.05
        self.nav_command_publisher.publish(message)

    def _cmd_vel_callback(self, message):
        self.cmd_vel_forwarded = self.cmd_vel_forwarded or (
            abs(message.linear.x) > 1.0e-4 or abs(message.angular.z) > 1.0e-4
        )

    def _thruster_callback(self, message):
        self.nonzero_thruster_command = self.nonzero_thruster_command or (
            len(message.data) == 4 and any(abs(value) > 1.0e-4 for value in message.data)
        )

    def complete(self):
        return all((
            self.map_to_odom,
            self.odom_to_base,
            self.waypoint_markers,
            self.valid_goal,
            self.cmd_vel_forwarded,
            self.nonzero_thruster_command,
        ))

    def failure_summary(self):
        missing = []
        checks = {
            "map->odom TF": self.map_to_odom,
            "odom->base_link TF": self.odom_to_base,
            "Task1 waypoint markers": self.waypoint_markers,
            "11-pose map-frame waypoint action": self.valid_goal,
            "/cmd_vel_nav forwarded to /cmd_vel": self.cmd_vel_forwarded,
            "four-channel nonzero /thruster_command": self.nonzero_thruster_command,
        }
        for name, passed in checks.items():
            if not passed:
                missing.append(name)
        if self.invalid_goal_reason:
            missing.append(self.invalid_goal_reason)
        return ", ".join(missing)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=45.0)
    arguments = parser.parse_args()

    rclpy.init()
    node = Task1HeadlessSmoke()
    deadline = time.monotonic() + arguments.timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline and not node.complete():
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.complete():
            raise RuntimeError(f"Task1 headless smoke timed out: {node.failure_summary()}")
        print("Task1 headless smoke passed")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
