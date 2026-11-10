from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("thruster_driver"),
        "config",
        "config.yaml",
    )
    default_robot_description = os.path.join(
        get_package_share_directory("robot"),
        "urdf",
        "robot.urdf.xacro",
    )

    config_file = LaunchConfiguration("config_file")
    robot_description_file = LaunchConfiguration("robot_description_file")
    use_velocity_feedback = LaunchConfiguration("use_velocity_feedback")
    stop_on_feedback_timeout = LaunchConfiguration("stop_on_feedback_timeout")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Path to thruster_driver config yaml",
            ),
            DeclareLaunchArgument(
                "robot_description_file",
                default_value=default_robot_description,
                description="Path to robot xacro/URDF containing fixed thruster poses",
            ),
            DeclareLaunchArgument(
                "use_velocity_feedback",
                default_value="true",
                description="Subscribe to feedback odometry and use measured velocity in control.",
            ),
            DeclareLaunchArgument(
                "stop_on_feedback_timeout",
                default_value="true",
                description="Force zero thruster output when /odometry/filtered/local "
                "is missing/stale. Set false for manual/bench testing without "
                "localization running.",
            ),
            Node(
                package="thruster_driver",
                executable="thruster_driver_node",
                name="thruster_driver_node",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "robot_description": ParameterValue(
                            Command(["xacro ", robot_description_file]),
                            value_type=str,
                        ),
                        "control.stop_on_feedback_timeout": ParameterValue(
                            stop_on_feedback_timeout,
                            value_type=bool,
                        ),
                        "control.use_velocity_feedback": ParameterValue(
                            use_velocity_feedback,
                            value_type=bool,
                        ),
                    },
                ],
            ),
        ]
    )
