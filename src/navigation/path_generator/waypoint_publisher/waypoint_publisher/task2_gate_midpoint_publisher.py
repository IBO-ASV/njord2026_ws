#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Quaternion
from nav2_msgs.action import NavigateThroughPoses


class Task2GateMidpointPublisher(Node):
    """
    Task2用のゲート中点生成ノード

    機能:
    1. 赤ブイ群 / 緑ブイ群のPoseArrayを購読する
    2. 各gate_xに最も近い赤ブイ・緑ブイをペアとして選ぶ
    3. 赤ブイと緑ブイの中点を計算する
    4. 中点を /task2/midpoints にPoseArrayとしてpublishする
    5. start -> 中点列 -> goal をNav2のNavigateThroughPosesへ送信する
    """

    def __init__(self):
        super().__init__("task2_gate_midpoint_publisher")

        # ============================================================
        # Parameters
        # ============================================================

        self.declare_parameter("frame_id", "map")

        # 入力: 赤ブイ・緑ブイのPoseArray
        self.declare_parameter("red_buoys_topic", "/task2/red_buoys")
        self.declare_parameter("green_buoys_topic", "/task2/green_buoys")

        # 出力: 計算した中点のPoseArray
        self.declare_parameter("midpoints_topic", "/task2/midpoints")

        # ゲートが存在すると期待するx座標
        # 例: gate_1 が x=20, gate_2 が x=40 付近
        self.declare_parameter("gate_xs", [20.0, 40.0])

        # start / goal
        self.declare_parameter("include_start", True)
        self.declare_parameter("start_x", 0.0)
        self.declare_parameter("start_y", 0.0)
        self.declare_parameter("goal_x", 60.0)
        self.declare_parameter("goal_y", 0.0)

        # gate_xから何mまで離れたブイを許容するか
        self.declare_parameter("max_gate_x_error", 10.0)

        # Nav2へ送信するか
        self.declare_parameter("send_to_nav2", True)

        # Nav2 action名
        self.declare_parameter("nav2_action_name", "/navigate_through_poses")

        self.frame_id = str(self.get_parameter("frame_id").value)

        self.red_buoys_topic = str(self.get_parameter("red_buoys_topic").value)
        self.green_buoys_topic = str(self.get_parameter("green_buoys_topic").value)
        self.midpoints_topic = str(self.get_parameter("midpoints_topic").value)

        self.gate_xs = [float(x) for x in self.get_parameter("gate_xs").value]

        self.include_start = bool(self.get_parameter("include_start").value)
        self.start_x = float(self.get_parameter("start_x").value)
        self.start_y = float(self.get_parameter("start_y").value)
        self.goal_x = float(self.get_parameter("goal_x").value)
        self.goal_y = float(self.get_parameter("goal_y").value)

        self.max_gate_x_error = float(self.get_parameter("max_gate_x_error").value)
        self.send_to_nav2 = bool(self.get_parameter("send_to_nav2").value)
        self.nav2_action_name = str(self.get_parameter("nav2_action_name").value)

        # ============================================================
        # Internal states
        # ============================================================

        self.red_buoys: Optional[PoseArray] = None
        self.green_buoys: Optional[PoseArray] = None

        self.nav_goal_requested = False
        self.nav_goal_accepted = False

        # ============================================================
        # Subscribers
        # ============================================================

        self.create_subscription(
            PoseArray,
            self.red_buoys_topic,
            self._red_buoys_callback,
            10,
        )

        self.create_subscription(
            PoseArray,
            self.green_buoys_topic,
            self._green_buoys_callback,
            10,
        )

        # ============================================================
        # Publisher
        # ============================================================
        # 中点は一度しかpublishされない可能性があるので、
        # ros2 topic echoを後から実行しても見えるようにTRANSIENT_LOCALにする。
        midpoint_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.midpoint_pub = self.create_publisher(
            PoseArray,
            self.midpoints_topic,
            midpoint_qos,
        )

        # ============================================================
        # Nav2 Action Client
        # ============================================================

        self.nav_client = ActionClient(
            self,
            NavigateThroughPoses,
            self.nav2_action_name,
        )

        # 1秒周期で確認
        self.timer = self.create_timer(1.0, self._timer_callback)

        self.get_logger().info("Task2GateMidpointPublisher initialized")
        self.get_logger().info(f"frame_id           : {self.frame_id}")
        self.get_logger().info(f"red_buoys_topic    : {self.red_buoys_topic}")
        self.get_logger().info(f"green_buoys_topic  : {self.green_buoys_topic}")
        self.get_logger().info(f"midpoints_topic    : {self.midpoints_topic}")
        self.get_logger().info(f"gate_xs            : {self.gate_xs}")
        self.get_logger().info(f"send_to_nav2       : {self.send_to_nav2}")
        self.get_logger().info(f"nav2_action_name   : {self.nav2_action_name}")

    # ============================================================
    # Callbacks
    # ============================================================

    def _red_buoys_callback(self, msg: PoseArray):
        self.red_buoys = msg

    def _green_buoys_callback(self, msg: PoseArray):
        self.green_buoys = msg

    def _timer_callback(self):
        """
        赤・緑ブイが揃ったら中点を計算し、
        /task2/midpoints にpublishし、
        必要ならNav2へ送信する。
        """

        if self.red_buoys is None or self.green_buoys is None:
            self.get_logger().warn("Waiting for red/green buoy poses...")
            return

        gate_midpoints = self._compute_gate_midpoints()

        if gate_midpoints is None:
            self.get_logger().warn("Failed to compute gate midpoints")
            return

        # 中点をトピックとしてpublish
        self._publish_midpoints(gate_midpoints)

        # Nav2へ送らない設定ならここで終了
        if not self.send_to_nav2:
            return

        # すでにNav2へgoal送信済みなら再送しない
        if self.nav_goal_requested:
            return

        # Nav2 action server確認
        if not self.nav_client.server_is_ready():
            self.get_logger().warn("NavigateThroughPoses action server not ready...")
            return

        nav2_poses = self._build_nav2_poses(gate_midpoints)

        self._send_goal(nav2_poses)
        self.nav_goal_requested = True

    # ============================================================
    # Midpoint calculation
    # ============================================================

    def _compute_gate_midpoints(self) -> Optional[List[Tuple[float, float]]]:
        """
        各gate_xに最も近い赤ブイ・緑ブイを選び、
        その中点を計算する。
        """

        if self.red_buoys is None or self.green_buoys is None:
            return None

        if len(self.red_buoys.poses) == 0:
            self.get_logger().warn("No red buoy poses received")
            return None

        if len(self.green_buoys.poses) == 0:
            self.get_logger().warn("No green buoy poses received")
            return None

        midpoints: List[Tuple[float, float]] = []

        for gate_x in self.gate_xs:
            red_pose = self._find_nearest_x_pose(self.red_buoys.poses, gate_x)
            green_pose = self._find_nearest_x_pose(self.green_buoys.poses, gate_x)

            if red_pose is None or green_pose is None:
                self.get_logger().warn(f"Could not find buoy pair near x={gate_x}")
                return None

            red_x = red_pose.position.x
            red_y = red_pose.position.y
            green_x = green_pose.position.x
            green_y = green_pose.position.y

            red_x_error = abs(red_x - gate_x)
            green_x_error = abs(green_x - gate_x)

            if red_x_error > self.max_gate_x_error:
                self.get_logger().warn(
                    f"Red buoy too far from expected gate x={gate_x}: "
                    f"red_x={red_x:.2f}, error={red_x_error:.2f}"
                )
                return None

            if green_x_error > self.max_gate_x_error:
                self.get_logger().warn(
                    f"Green buoy too far from expected gate x={gate_x}: "
                    f"green_x={green_x:.2f}, error={green_x_error:.2f}"
                )
                return None

            mid_x = 0.5 * (red_x + green_x)
            mid_y = 0.5 * (red_y + green_y)

            midpoints.append((mid_x, mid_y))

            self.get_logger().info(
                f"Gate midpoint near x={gate_x:.2f}: "
                f"red=({red_x:.2f}, {red_y:.2f}), "
                f"green=({green_x:.2f}, {green_y:.2f}), "
                f"mid=({mid_x:.2f}, {mid_y:.2f})"
            )

        # x座標順に並べる
        midpoints.sort(key=lambda p: p[0])

        return midpoints

    def _find_nearest_x_pose(self, poses, target_x: float) -> Optional[Pose]:
        """
        target_xに最も近いPoseを返す。
        """

        if len(poses) == 0:
            return None

        return min(
            poses,
            key=lambda pose: abs(pose.position.x - target_x),
        )

    # ============================================================
    # Publish midpoints
    # ============================================================

    def _publish_midpoints(self, gate_midpoints: List[Tuple[float, float]]):
        """
        計算した中点をPoseArrayとしてpublishする。
        トピック名はデフォルトで /task2/midpoints。
        """

        msg = PoseArray()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()

        for i, (x, y) in enumerate(gate_midpoints):
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = 0.0

            yaw = self._compute_yaw(gate_midpoints, i)
            pose.orientation = self._yaw_to_quaternion(yaw)

            msg.poses.append(pose)

        self.midpoint_pub.publish(msg)

        self.get_logger().info(
            f"Published {len(msg.poses)} gate midpoints to {self.midpoints_topic}"
        )

    # ============================================================
    # Build Nav2 poses
    # ============================================================

    def _build_nav2_poses(
        self,
        gate_midpoints: List[Tuple[float, float]],
    ) -> List[PoseStamped]:
        """
        start -> gate midpoints -> goal のPoseStamped列を作る。
        """

        points: List[Tuple[float, float]] = []

        if self.include_start:
            points.append((self.start_x, self.start_y))

        points.extend(gate_midpoints)
        points.append((self.goal_x, self.goal_y))

        poses: List[PoseStamped] = []

        for i, (x, y) in enumerate(points):
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.frame_id
            pose_stamped.header.stamp = self.get_clock().now().to_msg()

            pose_stamped.pose.position.x = float(x)
            pose_stamped.pose.position.y = float(y)
            pose_stamped.pose.position.z = 0.0

            yaw = self._compute_yaw(points, i)
            pose_stamped.pose.orientation = self._yaw_to_quaternion(yaw)

            poses.append(pose_stamped)

        return poses

    def _compute_yaw(self, points: List[Tuple[float, float]], index: int) -> float:
        """
        次の点へ向かう向きをyawとして設定する。
        最後の点では1つ前からの向きを使う。
        """

        if len(points) <= 1:
            return 0.0

        if index < len(points) - 1:
            x0, y0 = points[index]
            x1, y1 = points[index + 1]
        else:
            x0, y0 = points[index - 1]
            x1, y1 = points[index]

        return math.atan2(y1 - y0, x1 - x0)

    def _yaw_to_quaternion(self, yaw: float) -> Quaternion:
        """
        yaw角からQuaternionを作る。
        2D航行なのでroll, pitchは0。
        """

        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(yaw / 2.0)
        quat.w = math.cos(yaw / 2.0)
        return quat

    # ============================================================
    # Nav2 action
    # ============================================================

    def _send_goal(self, poses: List[PoseStamped]):
        """
        NavigateThroughPosesへgoalを送る。
        """

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses

        self.get_logger().info(
            f"Sending {len(poses)} poses to {self.nav2_action_name}"
        )

        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        """
        NavigateThroughPosesのgoalが受理されたか確認する。
        """

        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Failed to send NavigateThroughPoses goal: {e}")
            self.nav_goal_requested = False
            self.nav_goal_accepted = False
            return

        if not goal_handle.accepted:
            self.get_logger().error("NavigateThroughPoses goal rejected")
            self.nav_goal_requested = False
            self.nav_goal_accepted = False
            return

        self.nav_goal_accepted = True
        self.get_logger().info("NavigateThroughPoses goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._navigation_result_callback)

    def _navigation_result_callback(self, future):
        """
        Nav2の実行結果をログに出す。
        """

        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"Failed to get navigation result: {e}")
            return

        self.get_logger().info(
            f"NavigateThroughPoses finished with status: {result.status}"
        )


def main(args=None):
    rclpy.init(args=args)

    node = Task2GateMidpointPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Task2GateMidpointPublisher")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()