"""Single-Jetson regression bringup: jetson_bringup + minipc_bringup together.

Used to verify the pre-split, single-machine configuration still works.
GLIM/MID360S/ZED 2i stay on the "Jetson" role even in this combined launch,
so minipc_bringup's enable_glim stays false (jetson_bringup already runs
GLIM).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def include_launch(package_name, path_parts, launch_arguments=None):
    # Keep each sub-bringup's argument names (many of them generic, e.g.
    # enable_diagnostics) scoped to its own include so they cannot leak
    # into the sibling include.
    return GroupAction(
        scoped=True,
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
    default_thruster_config = os.path.join(
        get_package_share_directory("thruster_driver"), "config", "config.yaml"
    )
    default_thruster_urdf = os.path.join(
        get_package_share_directory("robot"), "urdf", "robot.urdf_modified.urdf"
    )

    jetson_launch = include_launch(
        "robot",
        ["launch", "jetson_bringup.launch.py"],
        {
            "lidar_model": LaunchConfiguration("lidar_model"),
            "enable_mid360": LaunchConfiguration("enable_mid360"),
            "enable_zed2i": LaunchConfiguration("enable_zed2i"),
            "enable_glim": LaunchConfiguration("enable_glim"),
            "glim_backend": LaunchConfiguration("glim_backend"),
            "enable_pcl_buoy_detection": LaunchConfiguration("enable_pcl_buoy_detection"),
            "enable_task1_default_buoy_roi": LaunchConfiguration("enable_task1_default_buoy_roi"),
            "enable_gpu_perception": LaunchConfiguration("enable_gpu_perception"),
            "engine_path": LaunchConfiguration("engine_path"),
            "camera_resolution": LaunchConfiguration("camera_resolution"),
            "camera_framerate": LaunchConfiguration("camera_framerate"),
            "enable_ground_video": LaunchConfiguration("enable_ground_video"),
            "ground_video_host": LaunchConfiguration("ground_video_host"),
            "ground_video_port": LaunchConfiguration("ground_video_port"),
            "heartbeat_monitor_zed2i": LaunchConfiguration("heartbeat_monitor_zed2i"),
            "heartbeat_monitor_lidar": LaunchConfiguration("heartbeat_monitor_lidar"),
            "enable_zenoh_bridge": LaunchConfiguration("enable_zenoh_bridge"),
            "lidar_start_delay": LaunchConfiguration("lidar_start_delay"),
            "camera_start_delay": LaunchConfiguration("camera_start_delay"),
        },
    )

    minipc_launch = include_launch(
        "robot",
        ["launch", "minipc_bringup.launch.py"],
        {
            "serial_port": LaunchConfiguration("serial_port"),
            "baud": LaunchConfiguration("baud"),
            "um982_port": LaunchConfiguration("um982_port"),
            "um982_protocol": LaunchConfiguration("um982_protocol"),
            "enable_um982_rtk": LaunchConfiguration("enable_um982_rtk"),
            "um982_feedback_mode": LaunchConfiguration("um982_feedback_mode"),
            "drogger_rzs_port": LaunchConfiguration("drogger_rzs_port"),
            "drogger_rzs_baud": LaunchConfiguration("drogger_rzs_baud"),
            "imu_port": LaunchConfiguration("imu_port"),
            "imu_baud": LaunchConfiguration("imu_baud"),
            "enable_um982": LaunchConfiguration("enable_um982"),
            "enable_drogger_rzs": LaunchConfiguration("enable_drogger_rzs"),
            "enable_imu": LaunchConfiguration("enable_imu"),
            "enable_localization": LaunchConfiguration("enable_localization"),
            "use_ekf_local": LaunchConfiguration("use_ekf_local"),
            "enable_thruster": LaunchConfiguration("enable_thruster"),
            "enable_alert_lamp": LaunchConfiguration("enable_alert_lamp"),
            "enable_bms": LaunchConfiguration("enable_bms"),
            "enable_buoy_costmap": LaunchConfiguration("enable_buoy_costmap"),
            "enable_nav2": LaunchConfiguration("enable_nav2"),
            "enable_control_manager": LaunchConfiguration("enable_control_manager"),
            "enable_mission_manager": LaunchConfiguration("enable_mission_manager"),
            "active_nav2_profile": LaunchConfiguration("active_nav2_profile"),
            "enable_diagnostics": LaunchConfiguration("enable_diagnostics"),
            "enable_zenoh_bridge": LaunchConfiguration("enable_zenoh_bridge"),
            "enable_critical_link": LaunchConfiguration("enable_critical_link"),
            "thruster_config_file": LaunchConfiguration("thruster_config_file"),
            "thruster_robot_description_file": LaunchConfiguration(
                "thruster_robot_description_file"
            ),
        },
    )

    return LaunchDescription(
        [
            # jetson_bringup.launch.py arguments
            DeclareLaunchArgument(
                "lidar_model", default_value="mid360s", choices=["mid360", "mid360s"]
            ),
            DeclareLaunchArgument("enable_mid360", default_value="true"),
            DeclareLaunchArgument("enable_zed2i", default_value="true"),
            DeclareLaunchArgument(
                "enable_glim",
                default_value="false",
                description="Load GLIM into the Livox component container on the Jetson side",
            ),
            DeclareLaunchArgument(
                "glim_backend", default_value="gpu", choices=["gpu", "cpu"]
            ),
            DeclareLaunchArgument("enable_pcl_buoy_detection", default_value="false"),
            DeclareLaunchArgument("enable_task1_default_buoy_roi", default_value="false"),
            DeclareLaunchArgument("enable_gpu_perception", default_value="true"),
            DeclareLaunchArgument("engine_path", default_value=default_engine),
            DeclareLaunchArgument("camera_resolution", default_value="HD720"),
            DeclareLaunchArgument("camera_framerate", default_value="15"),
            DeclareLaunchArgument("enable_ground_video", default_value="true"),
            DeclareLaunchArgument(
                "ground_video_host",
                default_value="osw-Stealth-14-AI-Studio-A1VGG.local",
            ),
            DeclareLaunchArgument("ground_video_port", default_value="5600"),
            DeclareLaunchArgument("heartbeat_monitor_zed2i", default_value="true"),
            DeclareLaunchArgument("heartbeat_monitor_lidar", default_value="true"),
            # The standalone regression launch contains both Jetson and miniPC
            # roles, so it must not start two bridge processes on one host.
            DeclareLaunchArgument("enable_zenoh_bridge", default_value="false"),
            DeclareLaunchArgument("enable_critical_link", default_value="false"),
            # Reproduce the staged sensor startup the pre-split
            # manual_control.launch.py used on the single Jetson.
            DeclareLaunchArgument("lidar_start_delay", default_value="18.0"),
            DeclareLaunchArgument("camera_start_delay", default_value="20.0"),
            # minipc_bringup.launch.py arguments
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
                "um982_port",
                default_value="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
            ),
            DeclareLaunchArgument("um982_protocol", default_value="uart"),
            DeclareLaunchArgument("enable_um982_rtk", default_value="false"),
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
            DeclareLaunchArgument(
                "use_ekf_local",
                default_value="false",
                description="Replace the default UM982 feedback EKF with the "
                "Livox-IMU local EKF.",
            ),
            DeclareLaunchArgument("enable_thruster", default_value="true"),
            DeclareLaunchArgument("enable_alert_lamp", default_value="true"),
            DeclareLaunchArgument("enable_bms", default_value="true"),
            DeclareLaunchArgument("enable_buoy_costmap", default_value="true"),
            DeclareLaunchArgument("enable_nav2", default_value="false"),
            DeclareLaunchArgument("enable_control_manager", default_value="true"),
            DeclareLaunchArgument("enable_mission_manager", default_value="true"),
            DeclareLaunchArgument(
                "active_nav2_profile", default_value="task1", choices=["task1", "task2", "task3"]
            ),
            DeclareLaunchArgument("enable_diagnostics", default_value="true"),
            DeclareLaunchArgument("thruster_config_file", default_value=default_thruster_config),
            DeclareLaunchArgument(
                "thruster_robot_description_file", default_value=default_thruster_urdf
            ),
            jetson_launch,
            minipc_launch,
        ]
    )
