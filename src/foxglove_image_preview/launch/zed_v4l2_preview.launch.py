"""Publish a low-latency left-eye JPEG preview from a ZED UVC stream."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    raw_topic = LaunchConfiguration("raw_topic")
    output_prefix = LaunchConfiguration("output_prefix")

    camera = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="zed2i_v4l2_camera",
        output="screen",
        remappings=[
            ("image_raw", raw_topic),
            ("camera_info", "/zed2i/stereo/camera_info"),
        ],
        parameters=[{
            "video_device": LaunchConfiguration("video_device"),
            "image_width": 2560,
            "image_height": 720,
            "framerate": 15.0,
            "io_method": "mmap",
            "pixel_format": "yuyv2rgb",
        }],
    )

    preview = Node(
        package="foxglove_image_preview",
        executable="preview_node",
        name="zed2i_foxglove_preview",
        output="screen",
        parameters=[{
            "input_topic": raw_topic,
            "output_prefix": output_prefix,
            "output_raw_topic": "/zed2i/left/image_raw",
            "width": 640,
            "height": 360,
            "max_fps": LaunchConfiguration("max_fps"),
            "jpeg_qualities": [40],
        }],
    )

    return LaunchDescription([
        # /dev/videoN numbering is not stable across reconnects.  Match the
        # left-eye device already recorded in zed2i_jetson_orin_nano.yaml so
        # the V4L2 fallback cannot silently select the rear camera instead.
        DeclareLaunchArgument(
            "video_device",
            default_value=(
                "/dev/v4l/by-id/usb-Technologies__Inc._ZED_2i_"
                "OV0001-video-index0"
            ),
        ),
        DeclareLaunchArgument("raw_topic", default_value="/zed2i/stereo/image_raw"),
        DeclareLaunchArgument("output_prefix", default_value="/zed2i/left/preview"),
        DeclareLaunchArgument("max_fps", default_value="10.0"),
        camera,
        preview,
    ])
