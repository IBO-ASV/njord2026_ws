"""Loop critical-link UDP back without publishing operational control topics."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="critical_link",
                executable="critical_link_receiver",
                name="critical_link_receiver_loopback",
                output="screen",
                parameters=[
                    {
                        "udp_paths": ["loopback|127.0.0.1|45100"],
                        "serial_device": "",
                        "joy_output_topic": "/critical_link/test/output/joy",
                        "heartbeat_output_topic": (
                            "/critical_link/test/output/heartbeat"
                        ),
                    }
                ],
            ),
            Node(
                package="critical_link",
                executable="critical_link_sender",
                name="critical_link_sender_loopback",
                output="screen",
                parameters=[
                    {
                        "udp_paths": [
                            "loopback|127.0.0.1|127.0.0.1|45100"
                        ],
                        "serial_device": "",
                    }
                ],
            ),
        ]
    )
