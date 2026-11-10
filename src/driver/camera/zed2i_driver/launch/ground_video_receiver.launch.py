"""Receive RTP/JPEG and publish it as a local CompressedImage topic."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration('port')
    jitter_latency_ms = LaunchConfiguration('jitter_latency_ms')
    topic = LaunchConfiguration('topic')
    return LaunchDescription(
        [
            DeclareLaunchArgument('port', default_value='5600'),
            DeclareLaunchArgument('jitter_latency_ms', default_value='50'),
            # This topic is intentionally absent from both Zenoh bridge allow
            # lists: video stays on the ground PC after RTP/JPEG reception.
            DeclareLaunchArgument('topic', default_value='/ground_video/image/compressed'),
            Node(
                package='zed2i_driver',
                executable='ground_receiver',
                name='ground_receiver',
                output='screen',
                parameters=[{
                    'port': port,
                    'jitter_latency_ms': jitter_latency_ms,
                    'topic': topic,
                }],
            ),
        ]
    )
