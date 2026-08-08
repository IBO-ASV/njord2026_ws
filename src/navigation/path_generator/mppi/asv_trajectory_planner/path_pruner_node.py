#!/usr/bin/env python3
import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Quaternion


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(0.5 * yaw)
    q.w = math.cos(0.5 * yaw)
    return q


def dist2(a, b) -> float:
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return dx * dx + dy * dy


class PathPrunerNode(Node):
    def __init__(self):
        super().__init__("path_pruner_node")

        self.declare_parameter("input_path_topic", "/planned_path")
        self.declare_parameter("output_path_topic", "/planned_path_pruned")
        self.declare_parameter("odom_topic", "/odom")

        # closest_idx の少し先から使う。0なら最近傍点から。
        self.declare_parameter("skip_points_after_closest", 0)

        # 先頭に現在位置を入れることで、Nav2が後ろの点へ戻ろうとするのを防ぐ。
        self.declare_parameter("prepend_current_pose", True)

        # Pathが短すぎるときの最低点数
        self.declare_parameter("min_output_points", 3)

        # 近すぎる点を軽く間引く距離
        self.declare_parameter("min_point_spacing_m", 0.5)

        self.input_path_topic = self.get_parameter("input_path_topic").value
        self.output_path_topic = self.get_parameter("output_path_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value

        self.skip_points_after_closest = int(self.get_parameter("skip_points_after_closest").value)
        self.prepend_current_pose = bool(self.get_parameter("prepend_current_pose").value)
        self.min_output_points = int(self.get_parameter("min_output_points").value)
        self.min_point_spacing_m = float(self.get_parameter("min_point_spacing_m").value)

        self.odom: Optional[Odometry] = None

        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.create_subscription(Path, self.input_path_topic, self.path_callback, 10)
        self.pub_path = self.create_publisher(Path, self.output_path_topic, 10)

        self.get_logger().info(
            f"path_pruner_node started. "
            f"{self.input_path_topic} -> {self.output_path_topic}, "
            f"odom={self.odom_topic}, "
            f"skip_points_after_closest={self.skip_points_after_closest}, "
            f"prepend_current_pose={self.prepend_current_pose}"
        )

    def odom_callback(self, msg: Odometry):
        self.odom = msg

    def current_pose(self) -> Optional[Tuple[float, float, float]]:
        if self.odom is None:
            return None

        p = self.odom.pose.pose.position
        yaw = yaw_from_quat(self.odom.pose.pose.orientation)
        return float(p.x), float(p.y), yaw

    def make_pose(self, x: float, y: float, yaw: float, header) -> PoseStamped:
        ps = PoseStamped()
        ps.header = header
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = 0.0
        ps.pose.orientation = quat_from_yaw(yaw)
        return ps

    def set_orientations_from_points(self, poses):
        if len(poses) < 2:
            return

        for i in range(len(poses) - 1):
            x0 = poses[i].pose.position.x
            y0 = poses[i].pose.position.y
            x1 = poses[i + 1].pose.position.x
            y1 = poses[i + 1].pose.position.y
            yaw = math.atan2(y1 - y0, x1 - x0)
            poses[i].pose.orientation = quat_from_yaw(yaw)

        poses[-1].pose.orientation = poses[-2].pose.orientation

    def thin_points(self, poses):
        if len(poses) <= 2:
            return poses

        out = [poses[0]]
        last = (
            poses[0].pose.position.x,
            poses[0].pose.position.y,
        )

        min_d2 = self.min_point_spacing_m * self.min_point_spacing_m

        for ps in poses[1:-1]:
            p = (ps.pose.position.x, ps.pose.position.y)
            if dist2(p, last) >= min_d2:
                out.append(ps)
                last = p

        out.append(poses[-1])
        return out

    def path_callback(self, msg: Path):
        pose = self.current_pose()
        if pose is None:
            return

        if len(msg.poses) < 2:
            return

        own_x, own_y, own_yaw = pose
        own = (own_x, own_y)

        # 1. raw path の中で現在位置に一番近い点を探す
        pts = [
            (ps.pose.position.x, ps.pose.position.y)
            for ps in msg.poses
        ]

        closest_idx = min(
            range(len(pts)),
            key=lambda i: dist2(pts[i], own),
        )

        start_idx = min(
            closest_idx + max(0, self.skip_points_after_closest),
            len(msg.poses) - 1,
        )

        suffix = list(msg.poses[start_idx:])

        # 2. 近すぎる点だけ軽く間引く
        suffix = self.thin_points(suffix)

        # 3. 先頭に現在位置を入れる
        out = Path()
        out.header = msg.header

        if self.prepend_current_pose:
            out.poses.append(
                self.make_pose(own_x, own_y, own_yaw, msg.header)
            )

        out.poses.extend(suffix)

        # 4. 点数が少なすぎる場合は、そのままraw path後半を使う
        if len(out.poses) < self.min_output_points:
            out.poses = []
            if self.prepend_current_pose:
                out.poses.append(
                    self.make_pose(own_x, own_y, own_yaw, msg.header)
                )
            out.poses.extend(list(msg.poses[closest_idx:]))

        # 5. orientationを隣接点方向に合わせる
        self.set_orientations_from_points(out.poses)

        self.pub_path.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PathPrunerNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
