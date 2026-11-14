from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='velocity_autotuner',
            executable='velocity_autotuner',
            name='velocity_autotuner',
            output='screen',
        ),
    ])
