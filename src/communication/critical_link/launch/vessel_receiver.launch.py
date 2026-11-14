from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("critical_link"), "config", "vessel_receiver.yaml"]
                ),
            ),
            Node(
                package="critical_link",
                executable="critical_link_receiver",
                name="critical_link_receiver",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
