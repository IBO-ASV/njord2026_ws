from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "publish_rate_hz",
            default_value="1.0",
            description="Waypoint NavSatFix publication rate in Hz.",
        ),
        Node(
            package="gnss_map_telemetry_test_publisher",
            executable="norway_waypoint_publisher",
            name="norway_waypoint_publisher",
            parameters=[{
                "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
            }],
        ),
    ])
