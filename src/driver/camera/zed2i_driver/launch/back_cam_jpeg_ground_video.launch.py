"""CPU JPEG/RTP sender for the miniPC-connected back camera."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("image_topic", default_value="/back_cam/image_raw"),
            DeclareLaunchArgument("host", default_value=""),
            DeclareLaunchArgument("port", default_value="5602"),
            DeclareLaunchArgument("fps", default_value="5.0"),
            Node(
                package="zed2i_driver",
                executable="back_cam_ground_video_streamer",
                name="back_cam_jpeg_ground_video_streamer",
                output="screen",
                parameters=[
                    {
                        "image_topic": LaunchConfiguration("image_topic"),
                        "host": LaunchConfiguration("host"),
                        "port": LaunchConfiguration("port"),
                        "fps": LaunchConfiguration("fps"),
                    }
                ],
            ),
        ]
    )
