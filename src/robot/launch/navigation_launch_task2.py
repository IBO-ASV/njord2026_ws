"""Task 2 final command graph on miniPC.

MPPI supplies paths from Jetson.  The command path is strictly Controller ->
Smoother -> /cmd_vel_nav.  Task 2 deliberately has no LiDAR collision-stop
stage: obstacle avoidance is performed by the Jetson MPPI path planner.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    log_level = LaunchConfiguration("log_level")
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=None,
            param_rewrites={"use_sim_time": use_sim_time, "autostart": autostart},
            convert_types=True,
        ),
        allow_substs=True,
    )
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    common = {
        "output": "screen",
        "parameters": [configured_params],
        "arguments": ["--ros-args", "--log-level", log_level],
    }

    return LaunchDescription([
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(
                get_package_share_directory("robot"),
                "config",
                "nav2_params_task2_humble.yaml",
            ),
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("log_level", default_value="info"),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            remappings=remappings + [("cmd_vel", "/cmd_vel_controller")],
            **common,
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            remappings=remappings + [
                ("cmd_vel", "/cmd_vel_controller"),
                ("cmd_vel_smoothed", "/cmd_vel_nav"),
            ],
            **common,
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                {"use_sim_time": use_sim_time, "autostart": autostart},
                {"node_names": ["controller_server", "velocity_smoother"]},
            ],
        ),
    ])
