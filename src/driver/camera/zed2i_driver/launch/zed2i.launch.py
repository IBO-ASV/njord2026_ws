import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    if mode not in ("cpu", "sdk"):
        raise RuntimeError("mode must be either 'cpu' or 'sdk'")

    plugin = (
        "zed2i_driver::CpuStereoNode"
        if mode == "cpu"
        else "zed2i_driver::SdkNode"
    )
    return [
        ComposableNodeContainer(
            package="rclcpp_components",
            executable="component_container_mt",
            name="zed2i_container",
            namespace="/",
            output="screen",
            composable_node_descriptions=[
                ComposableNode(
                    package="zed2i_driver",
                    plugin=plugin,
                    name="zed2i",
                    namespace=LaunchConfiguration("namespace"),
                    parameters=[
                        LaunchConfiguration("params_file"),
                        {
                            "enable_gpu_perception": LaunchConfiguration("enable_gpu_perception"),
                            "engine_path": LaunchConfiguration("engine_path"),
                            "enable_vessel_perception": LaunchConfiguration("enable_vessel_perception"),
                            "vessel_engine_path": LaunchConfiguration("vessel_engine_path"),
                            "vessel_confidence_threshold": LaunchConfiguration("vessel_confidence_threshold"),
                            "publish_debug_detections": LaunchConfiguration("publish_debug_detections"),
                            "camera_resolution": LaunchConfiguration("camera_resolution"),
                            "framerate": LaunchConfiguration("framerate"),
                            "aec_agc_enable": LaunchConfiguration("aec_agc_enable"),
                            "aec_agc_roi_enable": LaunchConfiguration("aec_agc_roi_enable"),
                            "aec_agc_roi_x_ratio": LaunchConfiguration("aec_agc_roi_x_ratio"),
                            "aec_agc_roi_y_ratio": LaunchConfiguration("aec_agc_roi_y_ratio"),
                            "aec_agc_roi_width_ratio": LaunchConfiguration("aec_agc_roi_width_ratio"),
                            "aec_agc_roi_height_ratio": LaunchConfiguration("aec_agc_roi_height_ratio"),
                            "fov_ellipse_enable": LaunchConfiguration("fov_ellipse_enable"),
                            "fov_ellipse_cx_ratio": LaunchConfiguration("fov_ellipse_cx_ratio"),
                            "fov_ellipse_cy_ratio": LaunchConfiguration("fov_ellipse_cy_ratio"),
                            "fov_ellipse_a_ratio": LaunchConfiguration("fov_ellipse_a_ratio"),
                            "fov_ellipse_b_ratio": LaunchConfiguration("fov_ellipse_b_ratio"),
                            "enable_ground_video": LaunchConfiguration("enable_ground_video"),
                            "ground_video_host": LaunchConfiguration("ground_video_host"),
                            "ground_video_port": LaunchConfiguration("ground_video_port"),
                            "ground_video_width": LaunchConfiguration("ground_video_width"),
                            "ground_video_height": LaunchConfiguration("ground_video_height"),
                            "ground_video_fps": LaunchConfiguration("ground_video_fps"),
                            "ground_video_jpeg_quality": LaunchConfiguration("ground_video_jpeg_quality"),
                            "ground_video_max_pending_frames": LaunchConfiguration("ground_video_max_pending_frames"),
                            "ground_video_mtu": LaunchConfiguration("ground_video_mtu"),
                            "ground_video_draw_detections": LaunchConfiguration("ground_video_draw_detections"),
                        },
                    ],
                    extra_arguments=[{"use_intra_process_comms": True}],
                )
            ],
        )
    ]


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory("zed2i_driver"), "config", "zed2i_jetson_orin_nano.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="sdk"),
            DeclareLaunchArgument("namespace", default_value="/zed2i"),
            DeclareLaunchArgument("params_file", default_value=default_params_file),
            DeclareLaunchArgument("enable_gpu_perception", default_value="false"),
            DeclareLaunchArgument("engine_path", default_value=""),
            DeclareLaunchArgument(
                "enable_vessel_perception", default_value="false",
                description="Run the optional YOLO-World boat TensorRT engine on the same ZED GPU image.",
            ),
            DeclareLaunchArgument(
                "vessel_engine_path", default_value="",
                description="TensorRT engine exported from YOLO-World with classes=[boat] and NMS enabled.",
            ),
            DeclareLaunchArgument("vessel_confidence_threshold", default_value="0.25"),
            DeclareLaunchArgument(
                "publish_debug_detections",
                default_value="false",
                description="Publish the GPU perception BuoyDetectionArray on "
                "detection_topic (from the params_file, default "
                "/buoy_detections_3d). Only takes effect when "
                "enable_gpu_perception:=true.",
            ),
            DeclareLaunchArgument(
                "camera_resolution",
                default_value="HD720",
                description="ZED camera resolution: HD2K, HD1080, HD720, or VGA",
            ),
            DeclareLaunchArgument("framerate", default_value="15"),
            DeclareLaunchArgument("aec_agc_enable", default_value="true"),
            DeclareLaunchArgument(
                "aec_agc_roi_enable", default_value="true",
                description="Use the AEC/AGC metering ROI instead of the full image.",
            ),
            DeclareLaunchArgument("aec_agc_roi_x_ratio", default_value="0.0"),
            DeclareLaunchArgument("aec_agc_roi_y_ratio", default_value="0.5"),
            DeclareLaunchArgument("aec_agc_roi_width_ratio", default_value="1.0"),
            DeclareLaunchArgument("aec_agc_roi_height_ratio", default_value="0.5"),
            DeclareLaunchArgument(
                "fov_ellipse_enable",
                default_value="false",
                description="Enable the human-FOV-like elliptical region of "
                "interest; outside the ellipse is skipped/invalidated across "
                "RGB, depth, points, GPU perception, and ground video.",
            ),
            DeclareLaunchArgument("fov_ellipse_cx_ratio", default_value="0.5"),
            DeclareLaunchArgument("fov_ellipse_cy_ratio", default_value="0.5"),
            DeclareLaunchArgument("fov_ellipse_a_ratio", default_value="0.5"),
            DeclareLaunchArgument("fov_ellipse_b_ratio", default_value="0.5"),
            DeclareLaunchArgument("enable_ground_video", default_value="false"),
            DeclareLaunchArgument("ground_video_host", default_value=""),
            DeclareLaunchArgument("ground_video_port", default_value="5600"),
            DeclareLaunchArgument("ground_video_width", default_value="360"),
            DeclareLaunchArgument("ground_video_height", default_value="240"),
            DeclareLaunchArgument("ground_video_fps", default_value="3.0"),
            DeclareLaunchArgument("ground_video_jpeg_quality", default_value="70"),
            DeclareLaunchArgument("ground_video_max_pending_frames", default_value="1"),
            DeclareLaunchArgument("ground_video_mtu", default_value="1200"),
            DeclareLaunchArgument(
                "ground_video_draw_detections", default_value="false",
                description="Overlay TensorRT YOLO bounding boxes on the ground video.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
