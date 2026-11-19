from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('velocity_autotuner'),
        'config',
        'velocity_autotuner.yaml',
    ])
    return LaunchDescription([
        Node(
            package='velocity_autotuner',
            executable='velocity_autotuner',
            name='velocity_autotuner',
            parameters=[config_file],
            output='screen',
        ),
    ])
