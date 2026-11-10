"""Task 1 simulation with simulated sensor and EKF processing enabled."""

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
                'use_sensor_noise': 'true',
                'use_local_ekf': 'true',
                'use_global_ekf': 'true',
                'use_navsat': 'true',
            }.items(),
        )
    ])
