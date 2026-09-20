"""Jetson-side bringup: MID360S LiDAR + GLIM + buoy detection, and ZED 2i.

Jetson Orin Nano Super hosts CUDA/TensorRT/ZED SDK and owns the USB ZED 2i
camera and the Ethernet-attached Livox MID360S. Nothing else (no TF
publisher, no EKF, no thrusters/micon, no joy) belongs in this file: those
live on the miniPC side (see minipc_bringup.launch.py).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def include_launch(package_name, path_parts, condition, launch_arguments=None):
    # Keep included launch files' generic argument names (e.g. camera_resolution)
    # scoped to this include so they cannot leak into sibling includes.
    return GroupAction(
        scoped=True,
        condition=condition,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare(package_name)] + path_parts)
                ),
                launch_arguments=(launch_arguments or {}).items(),
            )
        ],
    )


def generate_launch_description():
    default_engine = os.path.join(
        get_package_share_directory("robot"), "config", "yolo_model", "best.engine"
    )

    lidar_model = LaunchConfiguration("lidar_model")
    enable_mid360 = LaunchConfiguration("enable_mid360")
    enable_zed2i = LaunchConfiguration("enable_zed2i")
    zed_camera_backend = LaunchConfiguration("zed_camera_backend")
    enable_glim = LaunchConfiguration("enable_glim")
    glim_headless = LaunchConfiguration("glim_headless")
    glim_backend = LaunchConfiguration("glim_backend")
    enable_pcl_buoy_detection = LaunchConfiguration("enable_pcl_buoy_detection")
    enable_gpu_perception = LaunchConfiguration("enable_gpu_perception")
    engine_path = LaunchConfiguration("engine_path")
    enable_vessel_perception = LaunchConfiguration("enable_vessel_perception")
    vessel_engine_path = LaunchConfiguration("vessel_engine_path")
    vessel_confidence_threshold = LaunchConfiguration("vessel_confidence_threshold")
    camera_resolution = LaunchConfiguration("camera_resolution")
    camera_framerate = LaunchConfiguration("camera_framerate")
    enable_ground_video_h264 = LaunchConfiguration("enable_ground_video_h264")
    enable_ground_video_jpeg = LaunchConfiguration("enable_ground_video_jpeg")
    ground_video_host = LaunchConfiguration("ground_video_host")
    ground_video_port = LaunchConfiguration("ground_video_port")
    ground_video_width = LaunchConfiguration("ground_video_width")
    ground_video_height = LaunchConfiguration("ground_video_height")
    ground_video_fps = LaunchConfiguration("ground_video_fps")
    ground_video_draw_detections = LaunchConfiguration("ground_video_draw_detections")
    heartbeat_monitor_zed2i = LaunchConfiguration("heartbeat_monitor_zed2i")
    heartbeat_monitor_lidar = LaunchConfiguration("heartbeat_monitor_lidar")
    enable_task1_safety_points = LaunchConfiguration("enable_task1_safety_points")
    enable_task1_default_buoy_roi = LaunchConfiguration("enable_task1_default_buoy_roi")
    enable_task2_autonomy = LaunchConfiguration("enable_task2_autonomy")
    runtime_parameter_sync = Node(
        package="robot",
        executable="runtime_parameter_sync.py",
        name="runtime_parameter_sync",
        output="screen",
        parameters=[{"role": "jetson", "publish_updates": False}],
    )
    dock_wall_correction = Node(
        package="mission_manager",
        executable="dock_wall_correction_node",
        name="dock_wall_correction",
        output="screen",
    )

    # Keep raw LiDAR local to Jetson; only the small corrected berth poses are
    # bridged to the miniPC Mission Manager.
    dock_wall_correction = Node(
        package="mission_manager",
        executable="dock_wall_correction_node",
        name="dock_wall_correction",
        output="screen",
    )

    mid360_launch = include_launch(
        "robot",
        ["launch", "lidar.launch.py"],
        IfCondition(enable_mid360),
        {
            "lidar_model": lidar_model,
            "enable_buoy_detection": PythonExpression([
                "'", enable_pcl_buoy_detection, "' == 'true' or '",
                enable_task1_default_buoy_roi, "' == 'true'",
            ]),
            "enable_glim": enable_glim,
            "glim_headless": glim_headless,
            "glim_backend": glim_backend,
        },
    )

    zed2i_launch = include_launch(
        "zed2i_driver",
        ["launch", "zed2i.launch.py"],
        IfCondition(PythonExpression([
            "'", enable_zed2i, "' == 'true' and '",
            zed_camera_backend, "' == 'sdk'",
        ])),
        {
            "mode": "sdk",
            "enable_gpu_perception": enable_gpu_perception,
            "engine_path": engine_path,
            "enable_vessel_perception": enable_vessel_perception,
            "vessel_engine_path": vessel_engine_path,
            "vessel_confidence_threshold": vessel_confidence_threshold,
            "camera_resolution": camera_resolution,
            "framerate": camera_framerate,
            # Jetson JPEG uses nvvidconv + nvjpegenc (the Tegra NVJPG
            # hardware encoder). H.264 is deliberately not launched here:
            # its inter-frame latency causes visible image misalignment.
            "enable_ground_video": PythonExpression([
                "'", enable_ground_video_jpeg, "' == 'true' and '",
                enable_ground_video_h264, "' != 'true'",
            ]),
            "ground_video_codec": "jpeg",
            "ground_video_host": ground_video_host,
            "ground_video_port": ground_video_port,
            "ground_video_width": ground_video_width,
            "ground_video_height": ground_video_height,
            "ground_video_fps": ground_video_fps,
            "ground_video_jpeg_quality": LaunchConfiguration("ground_video_jpeg_quality"),
            "ground_video_draw_detections": ground_video_draw_detections,
        },
    )

    # The ZED SDK currently detects the camera but fails in sl::Camera::open()
    # with CAMERA STREAM FAILED TO START on this Jetson.  The UVC/V4L2 path is
    # independently verified and keeps only a bounded, low-bandwidth q40 JPEG
    # preview for ground/Foxglove use.
    zed_v4l2_preview_launch = include_launch(
        "foxglove_image_preview",
        ["launch", "zed_v4l2_preview.launch.py"],
        IfCondition(PythonExpression([
            "'", enable_zed2i, "' == 'true' and '",
            zed_camera_backend, "' == 'v4l2'",
        ])),
    )

    # Keep the annotated YOLO preview low-bandwidth before Zenoh forwards it.
    # This remains idle until a YOLO node publishes /yolo/debug_image.
    yolo_debug_preview = Node(
        package="foxglove_image_preview",
        executable="preview_node",
        name="yolo_foxglove_preview",
        output="screen",
        parameters=[{
            "input_topic": "/yolo/debug_image",
            "output_prefix": "/yolo/debug_preview",
            "split_stereo": False,
            "width": 640,
            "height": 360,
            "max_fps": 2.0,
            "jpeg_qualities": [40],
        }],
        condition=IfCondition(enable_zed2i),
    )

    heartbeat_launch = include_launch(
        "robot",
        ["launch", "heartbeat.launch.py"],
        IfCondition(LaunchConfiguration("enable_heartbeats")),
        {
            "role": "jetson",
            "heartbeat_monitor_zed2i": heartbeat_monitor_zed2i,
            "heartbeat_monitor_lidar": heartbeat_monitor_lidar,
        },
    )

    # Do not forward raw depth or LiDAR clouds to the miniPC. This compact,
    # base_link-frame cloud is generated from the ZED2i depth point cloud and
    # is the only near-field obstacle feed sent across the link for Task 1.
    task1_safety_points = Node(
        package="task2_perception",
        executable="task2_cloud_filter_node",
        name="task1_safety_points",
        output="screen",
        parameters=[{
            "input_topic": "/zed2i/points",
            "output_topic": "/task1/safety_points",
            "visual_output_topic": "/task1/safety_points_visual",
            "publish_visual_z_mirror": False,
            "output_frame": "base_link",
            "min_range_m": 0.5,
            "max_range_m": 8.0,
            "min_valid_input_points": 0,
            "publish_empty_on_invalid_input": True,
            "voxel_leaf_size_m": 0.25,
            "accumulation_frames": 1,
            "process_rate_hz": 5.0,
            "waterline_z_m": 0.0,
            "water_remove_min_z_m": -0.3,
            "water_remove_max_z_m": 0.15,
            "use_water_plane_ransac": False,
            "object_min_z_m": -0.5,
            "object_max_z_m": 2.0,
            "publish_self_marker": False,
            "publish_debug": False,
        }],
        condition=IfCondition(enable_task1_safety_points),
    )

    task1_default_buoy_roi = Node(
        package="task1_buoy_roi",
        executable="task1_buoy_roi",
        name="task1_buoy_roi",
        output="screen",
        condition=IfCondition(enable_task1_default_buoy_roi),
    )

    task2_autonomy = include_launch(
        "robot", ["launch", "task2_jetson_autonomy.launch.py"],
        IfCondition(enable_task2_autonomy),
        {"own_odom_topic": "/odometry/filtered/local", "enable_ship_tracking": "true"},
    )

    livox_gui_downsampler = Node(
        package="livox_gui_telemetry", executable="livox_gui_downsampler",
        name="livox_gui_downsampler", output="screen",
        condition=IfCondition(LaunchConfiguration("enable_livox_gui_telemetry")),
    )

    networking_launch = include_launch(
        "robot",
        ["launch", "networking.launch.py"],
        None,
        {
            "role": "jetson",
            "enable_zenoh_bridge": LaunchConfiguration("enable_zenoh_bridge"),
            "enable_critical_link": "false",
        },
    )

    # GLIM must always have the LiDAR's rigid extrinsic transform locally.
    # Do not depend on the miniPC's robot_state_publisher crossing Zenoh: a
    # late or missing /tf_static route otherwise leaves livox_frame unknown.
    # This topic remains local to the Jetson; bridge_jetson.json5 does not
    # export /tf_static, so it cannot duplicate the miniPC's shared TF edge.
    livox_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="jetson_livox_static_tf_pub",
        output="screen",
        arguments=[
            "--x", "0.5", "--y", "0.0", "--z", "0.8",
            "--roll", "3.141592653589793", "--pitch", "0.0", "--yaw", "0.0",
            "--frame-id", "base_link", "--child-frame-id", "livox_frame",
        ],
    )

    zed_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="jetson_zed_static_tf_pub",
        output="screen",
        arguments=[
            "--x", "0.48", "--y", "0.0", "--z", "0.47",
            "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
            "--frame-id", "base_link", "--child-frame-id", "zed2i_left_camera_frame",
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "lidar_model", default_value="mid360s", choices=["mid360", "mid360s"]
            ),
            DeclareLaunchArgument("enable_mid360", default_value="true"),
            DeclareLaunchArgument("enable_zed2i", default_value="true"),
            DeclareLaunchArgument(
                "zed_camera_backend",
                default_value="v4l2",
                choices=["v4l2", "sdk"],
                description=(
                    "Use the stable UVC q40 preview by default; select sdk to "
                    "restore depth, point cloud and GPU perception."
                ),
            ),
            DeclareLaunchArgument(
                "enable_glim",
                default_value="false",
                description="Load GLIM into the Livox component container",
            ),
            DeclareLaunchArgument(
                "glim_headless",
                default_value="true",
                description="Use GLIM's headless configuration without GUI viewer extensions.",
            ),
            DeclareLaunchArgument(
                "glim_backend",
                default_value="gpu",
                choices=["gpu", "cpu"],
                description="Select the GLIM config directory "
                "(glim_config for gpu, glim_config_cpu for cpu)",
            ),
            DeclareLaunchArgument(
                "enable_pcl_buoy_detection",
                default_value="false",
                description="Standalone LiDAR-only detector for explicit fallback testing. "
                "Normal operation uses ZED detection with its in-node LiDAR depth fallback.",
            ),
            DeclareLaunchArgument("enable_gpu_perception", default_value="true"),
            DeclareLaunchArgument("engine_path", default_value=default_engine),
            DeclareLaunchArgument("enable_vessel_perception", default_value="false"),
            DeclareLaunchArgument("vessel_engine_path", default_value=""),
            DeclareLaunchArgument("vessel_confidence_threshold", default_value="0.25"),
            DeclareLaunchArgument(
                "camera_resolution",
                default_value="HD720",
                description="ZED camera resolution: HD2K, HD1080, HD720, or VGA",
            ),
            DeclareLaunchArgument("camera_framerate", default_value="15"),
            DeclareLaunchArgument(
                "enable_ground_video_h264",
                default_value="false",
                description="H.264 ground-video is disabled; use the hardware JPEG stream instead.",
            ),
            DeclareLaunchArgument(
                "enable_ground_video_jpeg",
                default_value="true",
                description="Send ground video as JPEG via Jetson's nvjpegenc hardware encoder.",
            ),
            DeclareLaunchArgument(
                "ground_video_host",
                default_value="osw-Stealth-14-AI-Studio-A1VGG.local",
            ),
            DeclareLaunchArgument("ground_video_port", default_value="5600"),
            DeclareLaunchArgument("ground_video_width", default_value="360"),
            DeclareLaunchArgument("ground_video_height", default_value="240"),
            DeclareLaunchArgument("ground_video_fps", default_value="3.0"),
            DeclareLaunchArgument("ground_video_jpeg_quality", default_value="70"),
            DeclareLaunchArgument(
                "ground_video_draw_detections",
                default_value="true",
                description="Overlay the existing TensorRT YOLO detections on ground video.",
            ),
            DeclareLaunchArgument("enable_heartbeats", default_value="true"),
            DeclareLaunchArgument(
                "enable_task1_safety_points",
                default_value="true",
                description=(
                    "Publish compact near-field LiDAR safety points for the "
                    "miniPC Task 1 collision monitor."
                ),
            ),
            DeclareLaunchArgument(
                "enable_task1_default_buoy_roi",
                default_value="false",
                description="Use surveyed Task 1 buoy positions to publish /buoy_roi within 10 m.",
            ),
            DeclareLaunchArgument(
                "enable_task2_autonomy", default_value="false",
                description="Run Task 2 LiDAR perception, tracking and MPPI on Jetson.",
            ),
            DeclareLaunchArgument("enable_livox_gui_telemetry", default_value="true", description="Publish bounded Livox GUI telemetry only."),
            runtime_parameter_sync,
            livox_gui_downsampler,
            DeclareLaunchArgument(
                "enable_zenoh_bridge",
                default_value="true",
                description="Start the Jetson zenoh-bridge-ros2dds process.",
            ),
            DeclareLaunchArgument("heartbeat_monitor_zed2i", default_value="true"),
            DeclareLaunchArgument("heartbeat_monitor_lidar", default_value="true"),
            # Staged startup, carried over from manual_control.launch.py. On a
            # dedicated Jetson there is far less contention than in the old
            # single-machine setup, but starting the ZED/TensorRT stack after
            # the Livox container avoids simultaneous device/model startup;
            # standalone_bringup.launch.py passes the original 18.0/20.0 values
            # to reproduce the pre-split behaviour.
            DeclareLaunchArgument(
                "lidar_start_delay",
                default_value="0.5",
                description="Seconds to wait for the local livox static TF publisher before GLIM starts",
            ),
            DeclareLaunchArgument(
                "camera_start_delay",
                default_value="5.0",
                description="Seconds to wait before starting the ZED 2i driver",
            ),
            TimerAction(
                period=LaunchConfiguration("lidar_start_delay"),
                actions=[mid360_launch, dock_wall_correction],
            ),
            TimerAction(
                period=LaunchConfiguration("camera_start_delay"),
                actions=[zed2i_launch, zed_v4l2_preview_launch, yolo_debug_preview],
            ),
            livox_static_tf,
            zed_static_tf,
            TimerAction(period=0.5, actions=[task1_safety_points]),
            task1_default_buoy_roi,
            task2_autonomy,
            heartbeat_launch,
            networking_launch,
        ]
    )
