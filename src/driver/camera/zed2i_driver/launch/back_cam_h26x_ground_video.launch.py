"""Hardware H.264/H.265 RTP sender for the miniPC-connected back camera."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/back_cam/image_raw'),
        DeclareLaunchArgument('host', default_value=''),
        DeclareLaunchArgument('port', default_value='5601'),
        DeclareLaunchArgument('codec', default_value='h264', choices=['h264', 'h265']),
        DeclareLaunchArgument('fps', default_value='4.0'),
        DeclareLaunchArgument('width', default_value='480'),
        DeclareLaunchArgument('height', default_value='360'),
        DeclareLaunchArgument('bitrate_kbps', default_value='800'),
        DeclareLaunchArgument('keyframe_interval', default_value='5'),
        DeclareLaunchArgument('mtu', default_value='1200'),
        Node(
            package='zed2i_driver',
            executable='back_cam_h26x_ground_video_streamer',
            name='back_cam_h26x_ground_video_streamer',
            output='screen',
            parameters=[{
                'image_topic': LaunchConfiguration('image_topic'),
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
                'codec': LaunchConfiguration('codec'),
                'fps': LaunchConfiguration('fps'),
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
                'bitrate_kbps': LaunchConfiguration('bitrate_kbps'),
                'keyframe_interval': LaunchConfiguration('keyframe_interval'),
                'mtu': LaunchConfiguration('mtu'),
            }],
        ),
    ])
