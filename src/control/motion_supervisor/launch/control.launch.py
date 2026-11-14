"""Standalone canonical control stack; role bringup integration follows separately."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_policy = f"{get_package_share_directory('motion_supervisor')}/config/control_policy.yaml"
    policy_file = LaunchConfiguration("policy_file")
    return LaunchDescription([
        DeclareLaunchArgument("policy_file", default_value=default_policy),
        Node(package="motion_supervisor", executable="mode_manager", name="mode_manager", output="screen"),
        Node(
            package="motion_supervisor",
            executable="safety_supervisor",
            name="safety_supervisor",
            output="screen",
            parameters=[{"policy_file": policy_file}],
        ),
        Node(
            package="motion_supervisor",
            executable="command_arbiter",
            name="command_arbiter",
            output="screen",
            parameters=[{"command_timeout_sec": 2.0}],
        ),
    ])
