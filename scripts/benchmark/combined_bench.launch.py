# Combined intra-process bench: brings up the LiDAR/GLIM container
# (robot/launch/lidar.launch.py, bag-replay stand-in for the MID360) and the
# ZED2i SDK container (zed2i_driver/launch/zed2i.launch.py, GPU perception +
# ground-video transmit) CONCURRENTLY as two separate component containers,
# each internally using intra-process comms. Do not edit lidar.launch.py to
# add this; this file only composes the two existing launches.
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_engine_path = os.path.join(
        get_package_share_directory("robot"), "config", "yolo_model", "best.engine"
    )

    # GLIM's ROS publisher (rviz_viewer) resolves lidar_frame -> base_frame
    # before publishing /odom and /glim_node/points, and drops the frame when
    # the lookup fails. base_link comes from the URDF, so the bench must run
    # robot_state_publisher exactly like task1.launch.py does.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(
                        [
                            FindExecutable(name="xacro"),
                            " ",
                            PathJoinSubstitution(
                                [
                                    FindPackageShare("robot"),
                                    "urdf",
                                    "robot.urdf.xacro",
                                ]
                            ),
                        ]
                    ),
                    value_type=str,
                )
            }
        ],
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot"), "launch", "lidar.launch.py"
            )
        ),
        launch_arguments={
            "lidar_source": LaunchConfiguration("lidar_source"),
            "lidar_model": LaunchConfiguration("lidar_model"),
            # X11 is up, so run the real (viewer-enabled) GLIM config: the GUI
            # viewer's GPU cost is part of the load we want to measure.
            "glim_headless": LaunchConfiguration("glim_headless"),
            "bag_loop": "false",
            "enable_glim": "true",
            # pcl_det is retired from the pipeline; the LiDAR-side load in this
            # bench is GLIM only.
            "enable_buoy_detection": "false",
        }.items(),
    )

    zed2i_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("zed2i_driver"), "launch", "zed2i.launch.py"
            )
        ),
        launch_arguments={
            "mode": "sdk",
            "enable_gpu_perception": LaunchConfiguration("enable_gpu_perception"),
            "engine_path": LaunchConfiguration("engine_path"),
            "publish_debug_detections": LaunchConfiguration("enable_gpu_perception"),
            "enable_ground_video": LaunchConfiguration("enable_ground_video"),
            "ground_video_host": LaunchConfiguration("ground_video_host"),
        }.items(),
    )

    return LaunchDescription(
        [
            # Default to the real MID360S now that the sensor is on the wire
            # (192.168.1.114); lidar_source:=bag replays the recorded mcap.
            DeclareLaunchArgument("lidar_source", default_value="driver"),
            DeclareLaunchArgument("lidar_model", default_value="mid360s"),
            DeclareLaunchArgument("glim_headless", default_value="false"),
            DeclareLaunchArgument("enable_gpu_perception", default_value="true"),
            DeclareLaunchArgument("engine_path", default_value=default_engine_path),
            DeclareLaunchArgument("enable_ground_video", default_value="true"),
            DeclareLaunchArgument("ground_video_host", default_value="127.0.0.1"),
            robot_state_publisher,
            lidar_launch,
            zed2i_launch,
        ]
    )
