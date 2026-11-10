"""Mission-managed Task 2 simulation that starts immediately."""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("mission_manager"), "launch", "mission.launch.py"
        ])),
        launch_arguments={
            "runtime_mode": "task2_sim",
            "force_runtime_reconfigure": "true",
            "skip_coordinate_projection": "true",
        }.items(),
    )
    start_task = TimerAction(
        period=1.0,
        actions=[ExecuteProcess(
            cmd=["ros2", "run", "mission_manager", "njord-task", "start", "task2"],
            output="screen",
        )],
    )
    return LaunchDescription([mission, start_task])
