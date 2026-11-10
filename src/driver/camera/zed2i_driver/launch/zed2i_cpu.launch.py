import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory("zed2i_driver"), "config", "zed2i_cpu.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="/zed2i"),
            DeclareLaunchArgument("params_file", default_value=default_params_file),
            Node(
                package="zed2i_driver",
                executable="zed2i_cpu_node",
                name="zed2i",
                namespace=LaunchConfiguration("namespace"),
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                ],
            ),
        ]
    )
