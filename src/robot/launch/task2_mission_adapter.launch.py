"""miniPC half of Task 2: final Nav2 command and completion reporting only."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # MPPI, LiDAR filtering, segmentation and ship tracking run on Jetson.
    # Only the low-bandwidth Path crosses to this host; FollowPath remains
    # local because it owns the miniPC Nav2 action server and /cmd_vel_nav.
    waypoint_pose = Node(
        package="asv_trajectory_planner",
        executable="task2_waypoint_pose_publisher",
        name="task2_waypoint_pose_publisher",
        output="screen",
        parameters=[{
            "config_package": "waypoint_publisher", "config_file": "task2_waypoints.yaml",
            "config_key": "task2_config", "waypoint1_topic": "/waypoint1_pose",
            "waypoint2_topic": "/waypoint2_pose", "frame_id": "odom", "publish_frequency": 2.0,
        }],
    )
    follow_path = Node(
        package="asv_trajectory_planner",
        executable="follow_path_client_node",
        name="task2_follow_path_client",
        output="screen",
        parameters=[{
            "path_topic": "/planned_path_pruned",
            "action_name": "/follow_path",
            "controller_id": "FollowPath",
            "goal_checker_id": "general_goal_checker",
            "send_frequency": 1.0,
            "enable_replanning": True,
            "enabled_topic": "/mission/task2/enabled",
            "mission_gate_required": True,
            "goal_reached_topic": "/mission/task2/goal_reached",
        }],
    )
    return LaunchDescription([waypoint_pose, follow_path])
