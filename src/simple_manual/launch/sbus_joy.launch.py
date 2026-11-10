from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='/dev/ttyACM0'),
        Node(
            package='simple_manual',
            executable='sbus_joy_node',
            name='sbus_joy',
            output='screen',
            parameters=[{'device': LaunchConfiguration('device')}],
        ),
    ])
