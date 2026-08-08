"""miniPC-side bringup: vessel control and non-GPU sensors.

x86_64 Ubuntu 22.04 / ROS 2 Humble, no CUDA / no ZED SDK. Owns every USB
serial device that is enabled here (Micon and UM982 by default), localization
(robot_state_publisher, TF, the selected local filter, global EKF, and navsat_transform via
localization.launch.py), thrusters, and the back camera. The joy pad and
Foxglove bridge live on the ground PC. GLIM/MID360S/ZED 2i live on the
Jetson side (jetson_bringup.launch.py) and must not be duplicated here.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def include_launch(package_name, path_parts, condition, launch_arguments=None):
    # Included launch files commonly declare generic argument names (e.g.
    # serial_port, config_file). Keep those configurations local to the
    # include so one driver's launch args cannot leak into another's.
    return GroupAction(
        scoped=True,
        condition=condition,
        actions=[
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare(package_name)] + path_parts)
                ),
                launch_arguments=(launch_arguments or {}).items(),
            )
        ],
    )


def generate_launch_description():
    default_thruster_config = os.path.join(
        get_package_share_directory("thruster_driver"), "config", "config.yaml"
    )
    default_thruster_urdf = os.path.join(
        get_package_share_directory("robot"), "urdf", "robot.urdf_modified.urdf"
    )

    serial_port = LaunchConfiguration("serial_port")
    baud = LaunchConfiguration("baud")
    um982_port = LaunchConfiguration("um982_port")
    um982_protocol = LaunchConfiguration("um982_protocol")
    enable_um982_rtk = LaunchConfiguration("enable_um982_rtk")
    enable_spatial = LaunchConfiguration("enable_spatial")
    um982_feedback_mode = LaunchConfiguration("um982_feedback_mode")
    enable_um982 = LaunchConfiguration("enable_um982")
    enable_localization = LaunchConfiguration("enable_localization")
    enable_thruster = LaunchConfiguration("enable_thruster")
    enable_alert_lamp = LaunchConfiguration("enable_alert_lamp")
    alert_lamp_require_rtk_fix = LaunchConfiguration("alert_lamp_require_rtk_fix")
    enable_bms = LaunchConfiguration("enable_bms")
    enable_buoy_costmap = LaunchConfiguration("enable_buoy_costmap")
    enable_back_cam = LaunchConfiguration("enable_back_cam")
    enable_back_cam_ground_video = LaunchConfiguration("enable_back_cam_ground_video")
    back_cam_ground_video_host = LaunchConfiguration("back_cam_ground_video_host")
    back_cam_ground_video_port = LaunchConfiguration("back_cam_ground_video_port")
    back_cam_ground_video_codec = LaunchConfiguration("back_cam_ground_video_codec")
    back_cam_ground_video_fps = LaunchConfiguration("back_cam_ground_video_fps")
    back_cam_ground_video_width = LaunchConfiguration("back_cam_ground_video_width")
    back_cam_ground_video_height = LaunchConfiguration("back_cam_ground_video_height")
    enable_back_cam_jpeg_ground_video = LaunchConfiguration(
        "enable_back_cam_jpeg_ground_video"
    )
    back_cam_jpeg_ground_video_host = LaunchConfiguration(
        "back_cam_jpeg_ground_video_host"
    )
    back_cam_jpeg_ground_video_port = LaunchConfiguration(
        "back_cam_jpeg_ground_video_port"
    )
    back_cam_jpeg_ground_video_fps = LaunchConfiguration(
        "back_cam_jpeg_ground_video_fps"
    )
    enable_nav2 = LaunchConfiguration("enable_nav2")
    active_nav2_profile = LaunchConfiguration("active_nav2_profile")
    enable_control_manager = LaunchConfiguration("enable_control_manager")
    enable_mission_manager = LaunchConfiguration("enable_mission_manager")
    enable_diagnostics = LaunchConfiguration("enable_diagnostics")
    enable_autonomy_supervisor = LaunchConfiguration("enable_autonomy_supervisor")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_ekf_local = LaunchConfiguration("use_ekf_local")
    use_ekf_global = LaunchConfiguration("use_ekf_global")
    use_glim_fb = LaunchConfiguration("use_glim_fb")
    thruster_config_file = LaunchConfiguration("thruster_config_file")
    thruster_robot_description_file = LaunchConfiguration("thruster_robot_description_file")
    thruster_use_velocity_feedback = LaunchConfiguration("thruster_use_velocity_feedback")

    # robot_state_publisher and base_link->um982_link static TF are started by
    # localization.launch.py. Do not start them again here.
    localization_launch = include_launch(
        "robot",
        ["launch", "localization.launch.py"],
        IfCondition(enable_localization),
        {
            "enable_glim": "false",
            "enable_local_ekf": use_ekf_local,
            "enable_global_ekf": use_ekf_global,
            "enable_navsat_transform": "true",
            "enable_diagnostics": enable_diagnostics,
            "use_glim_fb": use_glim_fb,
            "use_sim_time": use_sim_time,
        },
    )

    um982_launch = include_launch(
        "um982_driver",
        ["launch", "um982.launch.py"],
        IfCondition(enable_um982),
        {
            "uart_or_tcp": um982_protocol,
            "gnss_port": um982_port,
            "rtk_enable": enable_um982_rtk,
            "ntrip_server": "osw-Stealth-14-AI-Studio-A1VGG.local",
            "ntrip_port": "2101",
            "ntrip_mountpoint": "RTCM3",
            "ntrip_username": "test",
            "ntrip_password": "test",
            "publish_feedback_odometry": "true",
        },
    )

    spatial_driver = Node(
        package="adnav_driver",
        executable="adnav_driver",
        name="adnav_driver",
        output="screen",
        emulate_tty=True,
        parameters=[
            PathJoinSubstitution([FindPackageShare("robot"), "config", "adnav_spatial.yaml"])
        ],
        condition=IfCondition(enable_spatial),
    )

    # The Spatial driver accepts RTCM through its NTRIP service and relays it
    # to the device as ANPP Packet 55.  The delay lets the serial driver finish
    # discovery and advertise the service before the one-shot configuration.
    spatial_ntrip = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/adnav_driver/ntrip",
                    "adnav_interfaces/srv/Ntrip",
                    "{enable: true, host: 'osw-Stealth-14-AI-Studio-A1VGG.local:2101', username: 'test', "
                    "password: 'test', mountpoint: 'RTCM3'}",
                ],
                output="screen",
            )
        ],
        condition=IfCondition(enable_spatial),
    )

    # Both filters publish /odometry/filtered/local.  The local Livox-IMU EKF
    # therefore replaces, rather than supplements, the UM982 feedback EKF.
    um982_feedback_launch = GroupAction(
        scoped=True,
        condition=UnlessCondition(use_ekf_local),
        actions=[
            include_launch(
                "um982_feedback_filter",
                ["launch", "um982_feedback.launch.py"],
                IfCondition(enable_um982),
                {
                    "feedback_mode": um982_feedback_mode,
                    "raw_topic": "/odometry/feedback",
                    "output_topic": "/odometry/filtered/local",
                },
            )
        ],
    )

    joy_converter = Node(
        package="simple_manual",
        executable="joy_converter_node",
        name="joy_converter",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("simple_manual"), "config", "joy_converter.yaml"]
            )
        ],
        # alert_lamp_driver is the sole publisher of the physical lamp topics.
        # Keep the joystick's button-status outputs available for debugging
        # without allowing them to contend with the alert-lamp state machine.
        remappings=[
            ("/green", "/manual_lamp/green"),
            ("/yellow", "/manual_lamp/yellow"),
            ("/red", "/manual_lamp/red"),
        ],
    )

    command_arbiter = Node(
        package="simple_manual",
        executable="command_arbiter_node",
        name="command_arbiter",
        output="screen",
        parameters=[{"initial_mode": "manual"}],
        # The legacy arbiter remains available only for comparison launches.
        # Normal role bringup has exactly one canonical /cmd_vel owner in
        # control_manager.
        condition=UnlessCondition(enable_control_manager),
    )

    control_manager_launch = include_launch(
        "control_manager",
        ["launch", "control.launch.py"],
        IfCondition(enable_control_manager),
    )

    mission_manager_launch = include_launch(
        "mission_manager",
        ["launch", "mission.launch.py"],
        IfCondition(enable_mission_manager),
        {"active_nav2_profile": active_nav2_profile},
    )

    thruster_launch = include_launch(
        "thruster_driver",
        ["launch", "thruster_driver.launch.py"],
        IfCondition(enable_thruster),
        {
            "config_file": thruster_config_file,
            "robot_description_file": thruster_robot_description_file,
            "use_velocity_feedback": thruster_use_velocity_feedback,
        },
    )

    thruster_serial = Node(
        package="micon_driver_fd",
        executable="thruster_serial",
        name="thruster_serial",
        output="screen",
        parameters=[
            {
                "serial_port": serial_port,
                "baud": baud,
                "command_topic": "/thruster_command",
                "ground_station_heartbeat_topic": "/heartbeat/ground_station",
                "ground_station_heartbeat_timeout_sec": 60.0,
                "use_sim_time": False,
            }
        ],
        condition=IfCondition(enable_thruster),
    )

    # The BMS master is a separate XIAO ESP32-C3 serial device. Its reader is
    # read-only: BMS telemetry cannot inject a thruster command or safety state.
    bms_serial = Node(
        package="micon_driver_fd",
        executable="bms_serial",
        name="bms_serial",
        output="screen",
        parameters=[
            {
                "serial_port": LaunchConfiguration("bms_serial_port"),
                "baud": LaunchConfiguration("bms_serial_baud"),
                "bms_topic": "/micon/bms_cells",
                "bms_temperature_topic": "/micon/bms_temperature_c",
                "ground_station_heartbeat_timeout_sec": 0.0,
                "use_sim_time": False,
            }
        ],
        condition=IfCondition(enable_bms),
    )

    bms_launch = include_launch(
        "bms",
        ["launch", "bms.launch.py"],
        IfCondition(enable_bms),
    )
    foxglove_logger = Node(
        package="foxglove_logger",
        executable="foxglove_logger_node",
        name="foxglove_logger",
        # output="screen",
    )

    alert_lamp_launch = include_launch(
        "alert_lamp",
        ["launch", "alert_lamp.launch.py"],
        IfCondition(enable_alert_lamp),
        {"require_rtk_fix": alert_lamp_require_rtk_fix},
    )

    buoy_obstacle_launch = include_launch(
        "buoy_obstacle_publisher",
        ["launch", "buoy_obstacle_publisher.launch.py"],
        IfCondition(enable_buoy_costmap),
    )

    back_cam_launch = include_launch(
        "robot",
        ["launch", "back_cam.launch.py"],
        IfCondition(enable_back_cam),
    )

    # The miniPC has a Radeon 780M VA-API encoder.  This is intentionally a
    # separate H.26x route from the Jetson's JPEG-only ground video path.
    back_cam_ground_video_launch = include_launch(
        "zed2i_driver",
        ["launch", "back_cam_h26x_ground_video.launch.py"],
        IfCondition(enable_back_cam_ground_video),
        {
            "host": back_cam_ground_video_host,
            "port": back_cam_ground_video_port,
            "codec": back_cam_ground_video_codec,
            "fps": back_cam_ground_video_fps,
            "width": back_cam_ground_video_width,
            "height": back_cam_ground_video_height,
        },
    )

    # Retain the CPU JPEG route as an explicit compatibility fallback. It is
    # disabled by default so the normal H.264 stream does not consume double
    # the radio bandwidth.
    back_cam_jpeg_ground_video_launch = include_launch(
        "zed2i_driver",
        ["launch", "back_cam_jpeg_ground_video.launch.py"],
        IfCondition(enable_back_cam_jpeg_ground_video),
        {
            "host": back_cam_jpeg_ground_video_host,
            "port": back_cam_jpeg_ground_video_port,
            "fps": back_cam_jpeg_ground_video_fps,
        },
    )

    nav2_launch = include_launch(
        "robot",
        ["launch", "nav2.launch.py"],
        IfCondition(enable_nav2),
        {"active_nav2_profile": active_nav2_profile},
    )

    # The supervisor owns the readiness and liveness signals consumed by the
    # command arbiter and alert lamp.  It can start before Nav2: it reports
    # ready only after the action server and a fresh waypoint plan appear.
    autonomy_supervisor = Node(
        package="diagnostic_monitors",
        executable="autonomy_supervisor_node",
        name="autonomy_supervisor",
        output="screen",
        condition=IfCondition(enable_autonomy_supervisor),
    )

    heartbeat_launch = include_launch(
        "robot",
        ["launch", "heartbeat.launch.py"],
        None,
        {"role": "minipc"},
    )

    networking_launch = include_launch(
        "robot",
        ["launch", "networking.launch.py"],
        None,
        {
            "role": "minipc",
            "enable_zenoh_bridge": LaunchConfiguration("enable_zenoh_bridge"),
            "enable_critical_link": LaunchConfiguration("enable_critical_link"),
        },
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_port",
                default_value=(
                    "/dev/serial/by-id/"
                    "usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_"
                    "c82421728a9aef118808b29061ce3355-if00-port0"
                ),
            ),
            DeclareLaunchArgument("baud", default_value="115200"),
            DeclareLaunchArgument(
                "bms_serial_port",
                default_value=(
                    "/dev/serial/by-id/"
                    "usb-Espressif_USB_JTAG_serial_debug_unit_E0:72:A1:1D:4B:14-if00"
                ),
                description="Stable /dev/serial/by-id path for the BMS ESP32-C3",
            ),
            DeclareLaunchArgument("bms_serial_baud", default_value="115200"),
            DeclareLaunchArgument(
                "um982_port",
                default_value="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
            ),
            DeclareLaunchArgument("um982_protocol", default_value="uart"),
            DeclareLaunchArgument(
                "enable_um982_rtk",
                default_value="true",
                description="Stream RTCM3 corrections from 192.168.1.72:2101 to UM982.",
            ),
            DeclareLaunchArgument(
                "enable_spatial",
                default_value="false",
                description="Start the miniPC-connected Advanced Navigation Spatial and enable NTRIP RTCM3.",
            ),
            DeclareLaunchArgument(
                "um982_feedback_mode", default_value="ekf", choices=["window", "ekf"]
            ),
            DeclareLaunchArgument(
                "drogger_rzs_port",
                default_value=(
                    "/dev/serial/by-id/"
                    "usb-Prolific_Technology_Inc._USB-Serial_Controller_"
                    "ACCQg146B12-if00-port0"
                ),
            ),
            DeclareLaunchArgument("drogger_rzs_baud", default_value="115200"),
            DeclareLaunchArgument("imu_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("imu_baud", default_value="9600"),
            DeclareLaunchArgument("enable_um982", default_value="true"),
            DeclareLaunchArgument("enable_drogger_rzs", default_value="false"),
            DeclareLaunchArgument("enable_imu", default_value="false"),
            DeclareLaunchArgument("enable_localization", default_value="true"),
            DeclareLaunchArgument("enable_thruster", default_value="true"),
            DeclareLaunchArgument("enable_alert_lamp", default_value="true"),
            DeclareLaunchArgument(
                "alert_lamp_require_rtk_fix",
                default_value="false",
                description="Require RTK Fix for alert-lamp AUTO readiness.",
            ),
            DeclareLaunchArgument("enable_bms", default_value="true"),
            DeclareLaunchArgument("enable_buoy_costmap", default_value="true"),
            DeclareLaunchArgument(
                "enable_back_cam",
                default_value="true",
                description="Start the miniPC-connected rear USB camera.",
            ),
            DeclareLaunchArgument(
                "enable_back_cam_ground_video",
                default_value="true",
                description="Use the miniPC VA-API H.264/H.265 back-camera stream. "
                "An empty host leaves the stream inactive.",
            ),
            DeclareLaunchArgument(
                "back_cam_ground_video_host",
                default_value="osw-Stealth-14-AI-Studio-A1VGG.local",
                description="Ground-station IPv4 address or Avahi .local hostname for "
                "the back-camera stream.",
            ),
            DeclareLaunchArgument("back_cam_ground_video_port", default_value="5601"),
            DeclareLaunchArgument(
                "back_cam_ground_video_codec", default_value="h264", choices=["h264", "h265"]
            ),
            DeclareLaunchArgument("back_cam_ground_video_fps", default_value="3.0"),
            DeclareLaunchArgument("back_cam_ground_video_width", default_value="360"),
            DeclareLaunchArgument("back_cam_ground_video_height", default_value="240"),
            DeclareLaunchArgument(
                "enable_back_cam_jpeg_ground_video",
                default_value="false",
                description="Optional CPU JPEG compatibility stream on UDP 5602.",
            ),
            DeclareLaunchArgument(
                "back_cam_jpeg_ground_video_host",
                default_value="osw-Stealth-14-AI-Studio-A1VGG.local",
            ),
            DeclareLaunchArgument(
                "back_cam_jpeg_ground_video_port", default_value="5602"
            ),
            DeclareLaunchArgument(
                "back_cam_jpeg_ground_video_fps", default_value="4.0"
            ),
            DeclareLaunchArgument(
                "use_glim_fb",
                default_value="false",
                description="Fuse Jetson GLIM /odom into the global EKF. "
                "Keep false when GLIM is disabled or cannot sustain real time.",
            ),
            DeclareLaunchArgument(
                "enable_nav2",
                default_value="true",
                description="Start the resident Nav2 graph owned by miniPC role bringup.",
            ),
            DeclareLaunchArgument(
                "enable_control_manager",
                default_value="true",
                description="Use the typed canonical control/safety stack instead of the legacy arbiter.",
            ),
            DeclareLaunchArgument(
                "enable_mission_manager",
                default_value="true",
                description="Start the persistent Mission Manager; it remains IDLE at startup.",
            ),
            DeclareLaunchArgument(
                "active_nav2_profile",
                default_value="task1",
                choices=["task1", "task2", "task3"],
                description="Resident Nav2 profile; incompatible Mission tasks are safely rejected.",
            ),
            DeclareLaunchArgument("enable_diagnostics", default_value="true"),
            DeclareLaunchArgument(
                "enable_zenoh_bridge",
                default_value="true",
                description="Start the miniPC zenoh-bridge-ros2dds process.",
            ),
            DeclareLaunchArgument(
                "enable_critical_link",
                default_value="true",
                description="Start critical_link_receiver for vessel control inputs.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use the /clock topic (set true when replaying a rosbag).",
            ),
            DeclareLaunchArgument(
                "enable_autonomy_supervisor",
                default_value="true",
                description="Publish /autonomy/ready and /heartbeat/autonomy from Nav2 and waypoint health.",
            ),
            DeclareLaunchArgument(
                "use_ekf_local",
                default_value="false",
                description="Replace the default UM982 feedback EKF with the "
                "Livox-IMU local EKF. The two filters are mutually exclusive.",
            ),
            DeclareLaunchArgument(
                "use_ekf_global",
                default_value="true",
                description="Enable the global EKF. The local EKF is always enabled.",
            ),
            DeclareLaunchArgument("thruster_config_file", default_value=default_thruster_config),
            DeclareLaunchArgument(
                "thruster_robot_description_file", default_value=default_thruster_urdf
            ),
            DeclareLaunchArgument(
                "thruster_use_velocity_feedback",
                default_value="true",
                description="Use measured local odometry velocity in thruster control.",
            ),
            localization_launch,
            um982_launch,
            spatial_driver,
            spatial_ntrip,
            um982_feedback_launch,
            # drogger_launch,
            # imu_node,
            joy_converter,
            command_arbiter,
            control_manager_launch,
            mission_manager_launch,
            thruster_launch,
            thruster_serial,
            bms_serial,
            bms_launch,
            foxglove_logger,
            alert_lamp_launch,
            buoy_obstacle_launch,
            back_cam_launch,
            back_cam_ground_video_launch,
            back_cam_jpeg_ground_video_launch,
            nav2_launch,
            autonomy_supervisor,
            heartbeat_launch,
            networking_launch,
        ]
    )
