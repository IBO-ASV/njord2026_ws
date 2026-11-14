"""The replaceable, profile-dependent miniPC Nav2 runtime."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    profile = LaunchConfiguration("profile")
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("robot"), "launch", "nav2.launch.py"])),
        launch_arguments={"profile": profile}.items(),
        condition=IfCondition(PythonExpression(["'", profile, "' != 'task2'"])))
    task2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("robot"), "launch", "navigation_launch_task2.py"])),
        condition=IfCondition(PythonExpression(["'", profile, "' == 'task2'"])))
    task2_adapter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("robot"), "launch", "task2_mission_adapter.launch.py"])),
        condition=IfCondition(PythonExpression(["'", profile, "' == 'task2'"])))
    return LaunchDescription([DeclareLaunchArgument("profile", choices=["task1", "task2", "task3"]), nav2, task2, task2_adapter])
