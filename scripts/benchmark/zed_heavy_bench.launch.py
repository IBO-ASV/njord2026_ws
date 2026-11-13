# ZED2i-ONLY heavy-load bench (NO LiDAR). Brings up just the ZED2i SDK
# container (zed2i_driver/launch/zed2i.launch.py) with a deliberately heavy
# configuration: HD1080@30, GPU perception enabled (same engine as
# combined_bench.launch.py), full-stride point cloud publish, and a heavier
# ground-video stream. Without LiDAR, /livox/lidar is absent so the fusion
# input to /buoy_detections_3d may go stale/empty -- that is acceptable; the
# purpose here is camera+depth+inference+encode load only. The node must not
# crash when no lidar is present.
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    default_engine_path = os.path.join(
        get_package_share_directory("robot"), "config", "yolo_model", "best.engine"
    )
    default_params_file = os.path.join(
        os.path.dirname(os.path.realpath(__file__)), "zed_heavy_params.yaml"
    )

    zed2i_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("zed2i_driver"), "launch", "zed2i.launch.py"
            )
        ),
        launch_arguments={
            "mode": "sdk",
            "params_file": LaunchConfiguration("params_file"),
            "camera_resolution": LaunchConfiguration("camera_resolution"),
            "framerate": LaunchConfiguration("framerate"),
            "fov_ellipse_enable": LaunchConfiguration("fov_ellipse_enable"),
            "fov_ellipse_a_ratio": LaunchConfiguration("fov_ellipse_a_ratio"),
            "fov_ellipse_b_ratio": LaunchConfiguration("fov_ellipse_b_ratio"),
            "enable_gpu_perception": LaunchConfiguration("enable_gpu_perception"),
            "engine_path": LaunchConfiguration("engine_path"),
            "publish_debug_detections": LaunchConfiguration("enable_gpu_perception"),
            "enable_ground_video": LaunchConfiguration("enable_ground_video"),
            "ground_video_host": LaunchConfiguration("ground_video_host"),
            "ground_video_width": LaunchConfiguration("ground_video_width"),
            "ground_video_height": LaunchConfiguration("ground_video_height"),
            "ground_video_fps": LaunchConfiguration("ground_video_fps"),
            "ground_video_jpeg_quality": LaunchConfiguration("ground_video_jpeg_quality"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params_file),
            DeclareLaunchArgument("camera_resolution", default_value="HD1080"),
            DeclareLaunchArgument("framerate", default_value="30"),
            DeclareLaunchArgument("fov_ellipse_enable", default_value="true"),
            DeclareLaunchArgument("fov_ellipse_a_ratio", default_value="0.5"),
            DeclareLaunchArgument("fov_ellipse_b_ratio", default_value="0.5"),
            DeclareLaunchArgument("enable_gpu_perception", default_value="true"),
            DeclareLaunchArgument("engine_path", default_value=default_engine_path),
            DeclareLaunchArgument("enable_ground_video", default_value="true"),
            DeclareLaunchArgument("ground_video_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("ground_video_width", default_value="1280"),
            DeclareLaunchArgument("ground_video_height", default_value="720"),
            DeclareLaunchArgument("ground_video_fps", default_value="15.0"),
            DeclareLaunchArgument("ground_video_jpeg_quality", default_value="90"),
            zed2i_launch,
        ]
    )
