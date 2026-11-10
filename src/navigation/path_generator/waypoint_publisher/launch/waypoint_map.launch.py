"""Display one task's surveyed waypoints in Foxglove without mission execution."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    task_type = LaunchConfiguration("task_type")
    return LaunchDescription([
        DeclareLaunchArgument(
            "task_type", default_value="task1",
            choices=["task1", "task1_skip_1_1", "task1_follow", "task2", "task2_bak", "task3_1", "task3_2", "task4"],
        ),
        DeclareLaunchArgument("marker_topic", default_value="/ground_waypoint_markers"),
        DeclareLaunchArgument("publish_rate_hz", default_value="1.0"),
        Node(
            package="waypoint_publisher", executable="ground_waypoint_geo_publisher",
            name="waypoint_map_publisher", output="screen",
            parameters=[{
                "task_type": ParameterValue(task_type, value_type=str),
                "marker_topic": LaunchConfiguration("marker_topic"),
                "publish_rate_hz": ParameterValue(
                    LaunchConfiguration("publish_rate_hz"), value_type=float),
            }],
        ),
    ])
