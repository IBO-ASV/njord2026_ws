"""Publish MID360S point clouds from the miniPC.

The Livox driver publishes ``sensor_msgs/msg/PointCloud2`` on
``/livox/lidar`` when ``xfer_format`` is 0.  This launch deliberately starts
only the driver, so it can be used on the miniPC without GLIM or other
Jetson-side perception nodes.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "user_config_path",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("robot"),
                        "config",
                        "livox",
                        "MID360S_minipc_config.json",
                    ]
                ),
                description="Path to the MID360S network configuration JSON",
            ),
            DeclareLaunchArgument("frame_id", default_value="livox_frame"),
            DeclareLaunchArgument("publish_freq", default_value="10.0"),
            Node(
                package="livox_ros_driver2",
                executable="livox_ros_driver2_node",
                name="livox_lidar_publisher",
                output="screen",
                parameters=[
                    {
                        # 0 selects sensor_msgs/PointCloud2 rather than the
                        # Livox custom message format.
                        "xfer_format": 0,
                        "multi_topic": 0,
                        "data_src": 0,
                        "publish_freq": LaunchConfiguration("publish_freq"),
                        "output_data_type": 0,
                        "frame_id": LaunchConfiguration("frame_id"),
                        "lvx_file_path": "/home/livox/livox_test.lvx",
                        "user_config_path": LaunchConfiguration("user_config_path"),
                        "cmdline_input_bd_code": "livox0000000001",
                    }
                ],
            ),
        ]
    )
