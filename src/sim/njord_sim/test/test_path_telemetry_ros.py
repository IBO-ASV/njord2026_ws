"""Late displays receive a retained path without idle retransmissions."""

import time

import rclpy
from nav_msgs.msg import Path
from njord_sim.environment import Environment, QOS


def spin(node, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)


def test_late_path_subscriber_and_idle_suppression():
    rclpy.init(args=[], domain_id=118)
    environment = viewer = None
    try:
        environment = Environment()
        spin(environment, 0.7)
        assert len(environment.actual_path.poses) == 1
        viewer = rclpy.create_node("late_path_viewer")
        messages = []
        viewer.create_subscription(Path, "/sim/estimated_path", messages.append, QOS)
        spin(viewer, 1.0)
        assert len(messages) == 1
        assert len(messages[-1].poses) == 1
        spin(environment, 1.0)
        spin(viewer, 0.3)
        assert len(messages) == 1
        environment.x += 0.2
        spin(environment, 0.6)
        spin(viewer, 0.3)
        assert len(messages) == 2
        assert len(messages[-1].poses) == 2
        first = environment.actual_path.poses[0]
        for _ in range(610):
            environment.x += 0.11
            environment.tick_count = 9
            environment.tick()
        assert len(environment.actual_path.poses) <= 600
        assert environment.actual_path.poses[0] == first
        assert environment.actual_path.poses[-1].pose.position.x == environment.x
        assert len({p.pose.position.x for p in environment.actual_path.poses}) == len(environment.actual_path.poses)
    finally:
        if viewer:
            viewer.destroy_node()
        if environment:
            environment.destroy_node()
        rclpy.shutdown()


def test_recognition_outage_keeps_physical_hull_and_truth():
    class Capture:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    rclpy.init(args=["--ros-args", "-p", "task:=task2"], domain_id=118)
    environment = None
    try:
        environment = Environment()
        environment.cloud = Capture()
        environment.lidar = Capture()
        environment.other_truth = Capture()
        environment.mission_start = time.monotonic() - 15.5
        stamp = environment.get_clock().now().to_msg()
        environment.publish_sensors(stamp, 15.5)
        opponent = environment.other_truth.messages[-1]
        environment.x = opponent.pose.pose.position.x
        environment.y = opponent.pose.pose.position.y
        environment.publish_sensors(stamp, 15.5)
        visible = environment.lidar.messages[-1].width
        physical = environment.cloud.messages[-1].width
        environment.detection_loss_period = 15.0
        environment.detection_loss_duration = 2.0
        environment.publish_sensors(stamp, 15.5)
        assert environment.lidar.messages[-1].width < visible - 100
        assert environment.cloud.messages[-1].width == physical
        assert len(environment.other_truth.messages) == 3
        assert environment.other_truth.messages[-1].twist.twist.linear.x > 0
    finally:
        if environment:
            environment.destroy_node()
        rclpy.shutdown()
