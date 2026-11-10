from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def include_launch(package_name, path_parts, condition, launch_arguments=None):
    # Included launch files commonly declare generic argument names such as
    # ``serial_port``. Keep those configurations local to the include so a
    # GNSS launch cannot overwrite the Micon serial port (or vice versa).
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
    enable_mid360 = LaunchConfiguration("enable_mid360")
    enable_pcl_buoy_detection = LaunchConfiguration("enable_pcl_buoy_detection")
    lidar_model = LaunchConfiguration("lidar_model")
    enable_zed2i = LaunchConfiguration("enable_zed2i")
    enable_gpu_perception = LaunchConfiguration("enable_gpu_perception")
    gpu_perception_engine_path = LaunchConfiguration("gpu_perception_engine_path")
    enable_ground_video = LaunchConfiguration("enable_ground_video")
    ground_video_host = LaunchConfiguration("ground_video_host")
    ground_video_port = LaunchConfiguration("ground_video_port")
    ground_video_fps = LaunchConfiguration("ground_video_fps")
    camera_resolution = LaunchConfiguration("camera_resolution")
    camera_framerate = LaunchConfiguration("camera_framerate")
    enable_back_cam = LaunchConfiguration("enable_back_cam")
    enable_um982 = LaunchConfiguration("enable_um982")
    enable_drogger_rzs = LaunchConfiguration("enable_drogger_rzs")
    enable_imu = LaunchConfiguration("enable_imu")
    enable_localization = LaunchConfiguration("enable_localization")
    enable_thruster = LaunchConfiguration("enable_thruster")
    enable_nav2 = LaunchConfiguration("enable_nav2")

    serial_port = LaunchConfiguration("serial_port")
    baud = LaunchConfiguration("baud")
    device = LaunchConfiguration("device")
    um982_protocol = LaunchConfiguration("um982_protocol")
    um982_port = LaunchConfiguration("um982_port")
    enable_um982_rtk = LaunchConfiguration("enable_um982_rtk")
    drogger_rzs_port = LaunchConfiguration("drogger_rzs_port")
    drogger_rzs_baud = LaunchConfiguration("drogger_rzs_baud")
    drogger_rzs_fix_topic = LaunchConfiguration("drogger_rzs_fix_topic")
    imu_port = LaunchConfiguration("imu_port")
    imu_baud = LaunchConfiguration("imu_baud")
    imu_frame_id = LaunchConfiguration("imu_frame_id")
    imu_publish_frequency = LaunchConfiguration("imu_publish_frequency")

    mid360_launch = include_launch(
        "robot",
        ["launch", "lidar.launch.py"],
        IfCondition(enable_mid360),
        {
            "lidar_model": lidar_model,
            "enable_buoy_detection": enable_pcl_buoy_detection,
            "enable_glim": LaunchConfiguration("enable_glim"),
            "glim_backend": LaunchConfiguration("glim_backend"),
        },
    )

    zed2i_launch = include_launch(
        "zed2i_driver",
        ["launch", "zed2i.launch.py"],
        IfCondition(enable_zed2i),
        {
            "mode": "sdk",
            "camera_resolution": camera_resolution,
            "framerate": camera_framerate,
            "enable_gpu_perception": enable_gpu_perception,
            "engine_path": gpu_perception_engine_path,
            "enable_ground_video": enable_ground_video,
            "ground_video_host": ground_video_host,
            "ground_video_port": ground_video_port,
            "ground_video_fps": ground_video_fps,
        },
    )

    back_cam_launch = include_launch(
        "robot",
        ["launch", "back_cam.launch.py"],
        IfCondition(enable_back_cam),
        {"video_device": device},
    )

    um982_launch = include_launch(
        "um982_driver",
        ["launch", "um982.launch.py"],
        IfCondition(enable_um982),
        {
            "uart_or_tcp": um982_protocol,
            "gnss_port": um982_port,
            "rtk_enable": enable_um982_rtk,
        },
    )

    drogger_rzs_launch = include_launch(
        "drogger_wired_flex",
        ["launch", "driver.launch.py"],
        IfCondition(enable_drogger_rzs),
        {
            "serial_port": drogger_rzs_port,
            "serial_baudrate": drogger_rzs_baud,
            "fix_topic": drogger_rzs_fix_topic,
        },
    )

    imu_node = Node(
        package="witmotion_imu_driver",
        executable="witmotion_standard_protocol_driver_node",
        name="witmotion_standard_protocol_driver",
        output="screen",
        parameters=[
            {
                "device_port_name": imu_port,
                "serial_baud_rate": imu_baud,
                "frame_id": imu_frame_id,
                "publish_frequency": imu_publish_frequency,
            }
        ],
        remappings=[("~/imu/raw", "/wit/imu")],
        condition=IfCondition(enable_imu),
    )

    localization_launch = include_launch(
        "robot",
        ["launch", "localization.launch.py"],
        IfCondition(enable_localization),
    )

    thruster_launch = include_launch(
        "thruster_driver",
        ["launch", "thruster_driver.launch.py"],
        IfCondition(enable_thruster),
        {"stop_on_feedback_timeout": "true"},
    )

    # TODO: micon_driver_fd currently has no standalone launch file; keep the
    # executable and node names here because manual_control.launch.py depends on them.
    serial_writer = Node(
        package="micon_driver_fd",
        executable="serial_writer",
        name="serial_writer",
        output="screen",
        parameters=[
            {
                "serial_port": serial_port,
                "baud": baud,
                "command_topic": "/thruster_command",
                "bms_topic": "micon/bms_cells",
                "use_sim_time": False,
            }
        ],
        condition=IfCondition(enable_thruster),
    )

    bms_launch = include_launch(
        "bms",
        ["launch", "bms.launch.py"],
        IfCondition(enable_thruster),
    )
    foxglove_logger = Node(
        package="foxglove_logger",
        executable="foxglove_logger_node",
        name="foxglove_logger",
        output="screen",
    )

    nav2_launch = include_launch(
        "robot",
        ["launch", "nav2.launch.py"],
        IfCondition(enable_nav2),
    )

    heading_arrow = Node(
        package="tf_frame_arrow_publisher",
        executable="arrow_publisher",
        name="nav_arrow_publisher",
        output="screen",
    )
    actual_route = Node(
        package="tf_frame_arrow_publisher",
        executable="full_path_publisher",
        name="actual_route_publisher",
        output="screen",
        parameters=[{
            "marker_topic": "/actual_path_marker",
            "parent_frame": "odom",
            "child_frame": "base_link",
        }],
    )

    return LaunchDescription(
        [
            # Livox LiDAR(mid360/mid360s)を起動するか
            DeclareLaunchArgument("enable_mid360", default_value="true"),
            DeclareLaunchArgument("enable_pcl_buoy_detection", default_value="false"),
            DeclareLaunchArgument(
                "lidar_model",
                default_value="mid360s",
                choices=["mid360", "mid360s"],
            ),
            DeclareLaunchArgument(
                "enable_glim",
                default_value="true",
                description="Load GLIM into the Livox component container",
            ),
            DeclareLaunchArgument(
                "glim_backend",
                default_value="gpu",
                choices=["gpu", "cpu"],
                description="Select the GLIM config directory (glim_config for gpu, glim_config_cpu for cpu)",
            ),
            DeclareLaunchArgument("enable_zed2i", default_value="true"),
            DeclareLaunchArgument("enable_gpu_perception", default_value="false"),
            DeclareLaunchArgument("gpu_perception_engine_path", default_value=""),
            DeclareLaunchArgument("enable_ground_video", default_value="false"),
            DeclareLaunchArgument(
                "ground_video_host",
                default_value="osw-Stealth-14-AI-Studio-A1VGG.local",
            ),
            DeclareLaunchArgument("ground_video_port", default_value="5600"),
            DeclareLaunchArgument("ground_video_fps", default_value="5.0"),
            DeclareLaunchArgument(
                "camera_resolution",
                default_value="HD720",
                description="ZED camera resolution: HD2K, HD1080, HD720, or VGA",
            ),
            DeclareLaunchArgument("camera_framerate", default_value="15"),
            DeclareLaunchArgument("enable_back_cam", default_value="true"),
            DeclareLaunchArgument("enable_um982", default_value="true"),
            DeclareLaunchArgument("enable_drogger_rzs", default_value="true"),
            DeclareLaunchArgument("enable_imu", default_value="false"),
            DeclareLaunchArgument("enable_localization", default_value="true"),
            DeclareLaunchArgument("enable_thruster", default_value="true"),
            DeclareLaunchArgument("enable_nav2", default_value="true"),
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_c82421728a9aef118808b29061ce3355-if00-port0",
            ),
            DeclareLaunchArgument("baud", default_value="115200"),
            DeclareLaunchArgument("um982_protocol", default_value="uart"),
            DeclareLaunchArgument("um982_port", default_value="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"),
            DeclareLaunchArgument("enable_um982_rtk", default_value="false"),
            DeclareLaunchArgument(
                "drogger_rzs_port",
                default_value="/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_ACCQg146B12-if00-port0",
            ),
            DeclareLaunchArgument("drogger_rzs_baud", default_value="115200"),
            DeclareLaunchArgument("drogger_rzs_fix_topic", default_value="/gnss/fix"),
            DeclareLaunchArgument("imu_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("imu_baud", default_value="9600"),
            DeclareLaunchArgument("imu_frame_id", default_value="imu_link"),
            DeclareLaunchArgument("imu_publish_frequency", default_value="10.0"),
            DeclareLaunchArgument(
                "device",
                default_value="/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._Adesso_CyberTrack_H7_SN0001-video-index0",
            ),
            heading_arrow,
            actual_route,
            mid360_launch,
            zed2i_launch,
            # back_cam_launch,
            um982_launch,
            drogger_rzs_launch,
            # imu_node,
            localization_launch,
            thruster_launch,
            serial_writer,
            bms_launch,
            foxglove_logger,
            # nav2_launch,
        ]
    )
