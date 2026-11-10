"""Persistent Mission Manager; it never submits a task at startup."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("active_nav2_profile", default_value=""),
        DeclareLaunchArgument("runtime_mode", default_value="nav2", choices=["nav2", "task2_sim"]),
        DeclareLaunchArgument("force_runtime_reconfigure", default_value="false"),
        DeclareLaunchArgument("skip_coordinate_projection", default_value="false"),
        DeclareLaunchArgument("waypoint_transform_enabled", default_value="false"),
        DeclareLaunchArgument("waypoint_transform_anchor_latitude", default_value="0.0"),
        DeclareLaunchArgument("waypoint_transform_anchor_longitude", default_value="0.0"),
        DeclareLaunchArgument("waypoint_transform_rotation_rad", default_value="0.0"),
        DeclareLaunchArgument("waypoint_transform_scale", default_value="1.0"),
        DeclareLaunchArgument(
            "ground_link_return_monitor_log_level", default_value="info"
        ),
        Node(
            package="mission_manager", executable="runtime_manager_node", name="runtime_manager", output="screen",
            parameters=[{
                "runtime_mode": LaunchConfiguration("runtime_mode"),
                "restart_on_configure": LaunchConfiguration("force_runtime_reconfigure"),
            }],
        ),
        Node(package="mission_manager", executable="operator_dispatcher_node", name="operator_dispatcher", output="screen"),
        Node(
            package="mission_manager",
            executable="ground_link_return_monitor_node",
            name="ground_link_return_monitor",
            output="screen",
            arguments=[
                "--ros-args",
                "--log-level",
                LaunchConfiguration("ground_link_return_monitor_log_level"),
            ],
        ),
        Node(
            package="mission_manager",
            executable="mission_manager_node",
            name="mission_manager",
            output="screen",
            parameters=[{
                "active_nav2_profile": LaunchConfiguration("active_nav2_profile"),
                "force_runtime_reconfigure": LaunchConfiguration("force_runtime_reconfigure"),
                "skip_coordinate_projection": LaunchConfiguration("skip_coordinate_projection"),
                "waypoint_transform_enabled": LaunchConfiguration("waypoint_transform_enabled"),
                "waypoint_transform_anchor_latitude": LaunchConfiguration("waypoint_transform_anchor_latitude"),
                "waypoint_transform_anchor_longitude": LaunchConfiguration("waypoint_transform_anchor_longitude"),
                "waypoint_transform_rotation_rad": LaunchConfiguration("waypoint_transform_rotation_rad"),
                "waypoint_transform_scale": LaunchConfiguration("waypoint_transform_scale"),
            }],
        ),
    ])
