"""Foxglove data/service endpoint shared by onboard and simulation launches."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("foxglove_address", default_value="0.0.0.0"),
            DeclareLaunchArgument("foxglove_port", default_value="8767"),
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="onboard_foxglove_bridge",
                output="screen",
                parameters=[
                    {
                        "address": LaunchConfiguration("foxglove_address"),
                        "port": ParameterValue(
                            LaunchConfiguration("foxglove_port"), value_type=int
                        ),
                        "capabilities": ["services", "parameters", "connectionGraph"],
                    }
                ],
            ),
        ]
    )
