"""Ground-station RTP/H.264 or RTP/H.265 receiver for the back camera."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='5601'),
        DeclareLaunchArgument('codec', default_value='h264', choices=['h264', 'h265']),
        DeclareLaunchArgument('jitter_latency_ms', default_value='50'),
        DeclareLaunchArgument('topic', default_value='/ground_video/back_cam/image_raw'),
        Node(
            package='zed2i_driver',
            executable='ground_h26x_receiver',
            name='ground_h26x_receiver',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'codec': LaunchConfiguration('codec'),
                'jitter_latency_ms': LaunchConfiguration('jitter_latency_ms'),
                'topic': LaunchConfiguration('topic'),
            }],
        ),
    ])
