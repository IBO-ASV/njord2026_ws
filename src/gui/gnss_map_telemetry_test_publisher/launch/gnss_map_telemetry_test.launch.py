from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="gnss_map_telemetry_test_publisher",
            executable="gnss_map_telemetry_test_publisher",
            name="gnss_map_telemetry_test_publisher",
            parameters=[{
                # Tokyo Bay. Heading is degrees clockwise from north.
                "latitude": 35.609596,
                "longitude": 139.683751,
                "heading_degrees": 90.0,
                "speed_mps": 2.0,
                "publish_rate_hz": 2.0,
            }],
        ),
    ])
