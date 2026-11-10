import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('bms'),
        'config',
        'config.yaml',
    )

    config_file = LaunchConfiguration('config_file')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'config_file',
                default_value=default_config,
                description='Path to bms config yaml',
            ),
            Node(
                package='bms',
                executable='bms_node',
                name='bms_node',
                output='screen',
                parameters=[config_file],
            ),
        ]
    )
