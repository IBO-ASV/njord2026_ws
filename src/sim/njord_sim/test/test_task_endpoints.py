"""Exercise ROS endpoint registration without starting navigation or hardware."""

import time

import pytest
import rclpy
from mission_manager.mission_manager_node import MissionManager
from njord_sim.configuration import configurations
from njord_sim.environment import QOS, Environment
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray


def spin_for(node, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)


@pytest.mark.parametrize("task", ["task1", "task2"])
def test_task_endpoints_and_repeated_task2_activation(task, tmp_path):
    _, _, registry = configurations(task, tmp_path)
    rclpy.init(args=["--ros-args", "-p", f"task:={task}",
                     "-p", f"registry_file:={registry}"], domain_id=119)
    environment = manager = None
    try:
        environment = Environment()
        manager = MissionManager()
        # A late-joining display must receive the preview while still IDLE,
        # before any navigation goal or MissionStatus callback has occurred.
        geographic, local = [], []
        manager.create_subscription(MarkerArray, "/ground_waypoint_markers", geographic.append, QOS)
        manager.create_subscription(MarkerArray, "/sim/waypoint_markers", local.append, QOS)
        deadline = time.monotonic() + 3
        while (not geographic or not local) and time.monotonic() < deadline:
            spin_for(manager, 0.05)
        assert geographic and local
        route = environment.s.route.waypoints
        geo = [m for m in geographic[-1].markers if m.action == Marker.ADD]
        assert len(geo) == len(route)
        assert geo[0].pose.position.x == route[0].longitude
        assert geo[0].pose.position.y == route[0].latitude
        assert geographic[-1].markers[0].action == Marker.DELETEALL
        assert local[-1].markers[0].action == Marker.DELETEALL
        assert all(m.header.frame_id == "odom" for m in local[-1].markers if m.action == Marker.ADD)
        assert sum(m.type == Marker.TEXT_VIEW_FACING for m in local[-1].markers) == len(route)
        topics = {subscription.topic_name for subscription in environment.subscriptions}
        task2_topics = {"/planned_path_pruned", "/task2/points_filtered",
                        "/mission/task2/enabled"}
        assert task2_topics.issubset(topics) if task == "task2" else not topics & task2_topics
        assert manager._task2_enabled_pub is None
        assert manager._task2_goal_sub is None
        if task == "task1":
            assert not manager._dock_nominal_pub
            return

        received = []
        manager._on_task2_goal_reached = lambda message: received.append(message.data)
        goal = environment.create_publisher(Bool, "/mission/task2/goal_reached", QOS)
        for _ in range(2):
            # A previous run's retained completion must not finish the new run.
            goal.publish(Bool(data=True))
            manager._set_task2_enabled(True)
            deadline = time.monotonic() + 3
            while goal.get_subscription_count() == 0 and time.monotonic() < deadline:
                spin_for(manager, 0.05)
            assert goal.get_subscription_count() > 0
            spin_for(manager, 0.15)
            assert received == []
            goal.publish(Bool(data=False))
            spin_for(manager, 0.15)
            goal.publish(Bool(data=True))
            spin_for(manager, 0.15)
            assert received == [False, True]
            manager._set_task2_enabled(False)
            assert manager._task2_goal_sub is None
            assert "/mission/task2/goal_reached" not in {
                subscription.topic_name for subscription in manager.subscriptions
            }
            received.clear()
    finally:
        if manager is not None:
            manager.destroy_node()
        if environment is not None:
            environment.destroy_node()
        rclpy.shutdown()
