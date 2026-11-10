"""Task 1 simulation using SimNode truth odometry only."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('task1_sim'), 'launch', 'task1_sim.launch.py'])
            ),
            launch_arguments={
                'use_sensor_noise': 'false',
                'use_local_ekf': 'false',
                'use_global_ekf': 'false',
                'use_navsat': 'false',
            }.items(),
        )
    ])
