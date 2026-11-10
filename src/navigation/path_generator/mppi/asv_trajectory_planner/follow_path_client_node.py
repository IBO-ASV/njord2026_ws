#!/usr/bin/env python3
import math
import time
from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
from action_msgs.msg import GoalStatus
from std_msgs.msg import Bool


class FollowPathClientNode(Node):
    """
    /planned_path などの nav_msgs/Path を Nav2 の FollowPath action に送るノード。
    """

    def __init__(self):
        super().__init__("follow_path_client_node")

        self.declare_parameter("path_topic", "/planned_path")
        self.declare_parameter("action_name", "/follow_path")
        self.declare_parameter("controller_id", "FollowPath")
        self.declare_parameter("goal_checker_id", "general_goal_checker")
        self.declare_parameter("send_frequency", 2.0)
        self.declare_parameter("enable_replanning", True)
        self.declare_parameter("enabled_topic", "/mission/task2/enabled")
        self.declare_parameter("mission_gate_required", False)
        self.declare_parameter("startup_hold_sec", 0.0)
        self.declare_parameter("goal_reached_topic", "/mission/task2/goal_reached")
        self.declare_parameter("final_goal_pose_topic", "/waypoint2_pose")
        self.declare_parameter("final_goal_endpoint_tolerance_m", 1.0)

        # goal active中にFollowPath goalを投げ直す最短間隔。
        # 短すぎるとcontroller_serverがabortしやすい。
        self.declare_parameter("active_replan_interval_s", 1.0)

        self.path_topic = self.get_parameter("path_topic").value
        self.action_name = self.get_parameter("action_name").value
        self.controller_id = self.get_parameter("controller_id").value
        self.goal_checker_id = self.get_parameter("goal_checker_id").value
        self.send_frequency = float(self.get_parameter("send_frequency").value)
        self.enable_replanning = bool(self.get_parameter("enable_replanning").value)
        self.enabled_topic = self.get_parameter("enabled_topic").value
        self.mission_gate_required = bool(
            self.get_parameter("mission_gate_required").value
        )
        self.startup_hold_sec = float(
            self.get_parameter("startup_hold_sec").value
        )
        if self.startup_hold_sec < 0.0:
            raise ValueError("startup_hold_sec must be non-negative")
        self.active_replan_interval_s = float(
            self.get_parameter("active_replan_interval_s").value
        )
        self.goal_reached_topic = str(self.get_parameter("goal_reached_topic").value)
        self.final_goal_pose_topic = str(self.get_parameter("final_goal_pose_topic").value)
        self.final_goal_endpoint_tolerance_m = float(
            self.get_parameter("final_goal_endpoint_tolerance_m").value
        )

        self.latest_path = None
        self.last_sent_path = None
        self.goal_active = False
        self.enabled = not self.mission_gate_required
        self.goal_handle = None
        self.last_goal_send_time = None
        self.goal_completed = False
        self.goal_generation = 0
        self.final_goal_pose = None
        self.startup_hold_until = None

        status_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.goal_reached_pub = self.create_publisher(Bool, self.goal_reached_topic, status_qos)
        self.goal_reached_pub.publish(Bool(data=False))

        self.path_sub = self.create_subscription(
            Path,
            self.path_topic,
            self.path_callback,
            10,
        )
        self.enabled_sub = self.create_subscription(
            Bool, self.enabled_topic, self.enabled_callback, status_qos
        )
        self.final_goal_sub = self.create_subscription(
            PoseStamped, self.final_goal_pose_topic, self.final_goal_callback, 10
        )

        self.action_client = ActionClient(
            self,
            FollowPath,
            self.action_name,
        )

        timer_period = 1.0 / max(self.send_frequency, 1e-6)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(f"Subscribe path: {self.path_topic}")
        self.get_logger().info(f"Action name   : {self.action_name}")
        self.get_logger().info(f"controller_id : {self.controller_id}")
        self.get_logger().info(f"goal_checker  : {self.goal_checker_id}")
        self.get_logger().info(f"send_frequency: {self.send_frequency}")
        self.get_logger().info(f"replanning    : {self.enable_replanning}")
        self.get_logger().info(
            f"Mission gate: {'required' if self.mission_gate_required else 'disabled'} "
            f"({self.enabled_topic})"
        )
        if self.mission_gate_required and self.startup_hold_sec > 0.0:
            self.get_logger().info(
                f"Task 2 startup hold: {self.startup_hold_sec:.1f}s "
                "with no autonomous propulsion"
            )
        self.get_logger().info(f"active interval: {self.active_replan_interval_s}")
        self.get_logger().info(
            f"Task 2 final goal: {self.final_goal_pose_topic} "
            f"(endpoint tolerance {self.final_goal_endpoint_tolerance_m:.2f} m)"
        )

    def path_callback(self, msg: Path):
        if len(msg.poses) < 2:
            self.get_logger().warn("Received path has fewer than 2 poses. Ignored.")
            return

        self.latest_path = deepcopy(msg)

    def final_goal_callback(self, msg: PoseStamped):
        self.final_goal_pose = deepcopy(msg)

    def enabled_callback(self, msg: Bool):
        if not self.mission_gate_required:
            return
        if self.enabled == msg.data:
            return
        self.enabled = msg.data
        self.goal_completed = False
        self.goal_reached_pub.publish(Bool(data=False))
        if not self.enabled:
            self.startup_hold_until = None
            self.goal_generation += 1
            self.latest_path = None
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            self.goal_active = False
            self.goal_handle = None
            self.get_logger().info("Task 2 Mission gate disabled; canceled FollowPath goal")
        else:
            self.startup_hold_until = (
                time.monotonic() + self.startup_hold_sec
                if self.startup_hold_sec > 0.0 else None
            )
            self.get_logger().info(
                "Task 2 Mission gate enabled; "
                + (f"holding autonomous propulsion for {self.startup_hold_sec:.1f}s"
                   if self.startup_hold_until is not None else "ready to send FollowPath goals")
            )

    def path_changed_enough(self, path_a: Path, path_b: Path) -> bool:
        if path_a is None or path_b is None:
            return True

        if len(path_a.poses) != len(path_b.poses):
            return True

        if len(path_a.poses) == 0:
            return False

        # first / middle / last の位置差で簡易判定
        ids = [0, len(path_a.poses) // 2, len(path_a.poses) - 1]

        for i in ids:
            ax = path_a.poses[i].pose.position.x
            ay = path_a.poses[i].pose.position.y
            bx = path_b.poses[i].pose.position.x
            by = path_b.poses[i].pose.position.y

            if math.hypot(ax - bx, ay - by) > 0.3:
                return True

        return False

    def timer_callback(self):
        if not self.enabled or self.goal_completed or self.latest_path is None:
            return

        if self.startup_hold_until is not None:
            remaining = self.startup_hold_until - time.monotonic()
            if remaining > 0.0:
                self.get_logger().info(
                    f"Task 2 buoy-observation hold: {remaining:.1f}s remaining",
                    throttle_duration_sec=1.0,
                )
                return
            self.startup_hold_until = None
            self.get_logger().info(
                "Task 2 buoy-observation hold complete; enabling FollowPath goals"
            )

        if not self.action_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn(
                f"Waiting for FollowPath action server: {self.action_name}",
                throttle_duration_sec=2.0,
            )
            return

        now = self.get_clock().now()

        if self.goal_active:
            if not self.enable_replanning:
                return

            if self.last_goal_send_time is not None:
                dt = (now - self.last_goal_send_time).nanoseconds * 1e-9
                if dt < self.active_replan_interval_s:
                    return

            if not self.path_changed_enough(self.latest_path, self.last_sent_path):
                return

        self.send_goal(self.latest_path)

    def send_goal(self, path: Path):
        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.controller_id = self.controller_id
        goal_msg.goal_checker_id = self.goal_checker_id

        self.get_logger().info(
            f"Sending FollowPath goal: {len(path.poses)} poses"
        )

        self.last_sent_path = deepcopy(path)
        self.last_goal_send_time = self.get_clock().now()

        self.goal_generation += 1
        generation = self.goal_generation
        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )
        send_goal_future.add_done_callback(
            lambda future: self.goal_response_callback(future, generation)
        )

        self.goal_active = True

    def goal_response_callback(self, future, generation):
        if generation != self.goal_generation:
            return
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("FollowPath goal was rejected.")
            self.goal_active = False
            return

        if not self.enabled:
            # The gate can close while send_goal_async is in flight.  Do not
            # retain a goal that was accepted after Mission cancellation.
            goal_handle.cancel_goal_async()
            self.goal_active = False
            return

        self.get_logger().info("FollowPath goal was accepted.")
        self.goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future: self.get_result_callback(future, generation)
        )

    def feedback_callback(self, feedback_msg):
        # 必要ならdebug表示
        _ = feedback_msg.feedback

    def get_result_callback(self, future, generation):
        if generation != self.goal_generation:
            return
        result_msg = future.result()
        status = result_msg.status
        result = result_msg.result

        if hasattr(result, "error_code"):
            self.get_logger().warn(
                f"FollowPath finished. status={status}, error_code={result.error_code}"
            )
        else:
            self.get_logger().warn(
                f"FollowPath finished. status={status}"
            )

        self.goal_active = False
        self.goal_handle = None
        if self.enabled and status == GoalStatus.STATUS_SUCCEEDED:
            if not self._path_ends_at_final_goal():
                # MPPI publishes a rolling horizon while avoiding an obstacle.
                # Completing such an intermediate FollowPath must not end Task2
                # or remove its Mission gate before GPS6 is reached.
                self.get_logger().info(
                    "FollowPath reached an intermediate MPPI horizon; "
                    "waiting for a path terminating at Task2 GPS6"
                )
                return
            self.goal_completed = True
            self.goal_reached_pub.publish(Bool(data=True))
            self.get_logger().info("Task 2 goal reached; reported to Mission Manager")

    def _path_ends_at_final_goal(self) -> bool:
        if self.last_sent_path is None or not self.last_sent_path.poses or self.final_goal_pose is None:
            return False
        endpoint = self.last_sent_path.poses[-1].pose.position
        goal = self.final_goal_pose.pose.position
        return math.hypot(endpoint.x - goal.x, endpoint.y - goal.y) <= (
            self.final_goal_endpoint_tolerance_m
        )


def main(args=None):
    rclpy.init(args=args)
    node = FollowPathClientNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
