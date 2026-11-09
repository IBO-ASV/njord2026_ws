"""Bring up the real vessel for joystick control without navigation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def include_launch(package_name, path_parts, condition=None, launch_arguments=None):
    """Include a launch file without leaking its generic launch arguments."""
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
    enable_detection = LaunchConfiguration('enable_detection')

    serial_port = LaunchConfiguration('serial_port')
    baud = LaunchConfiguration('baud')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'serial_port',
                default_value=(
                    '/dev/serial/by-id/'
                    'usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_'
                    'c82421728a9aef118808b29061ce3355-if00-port0'
                ),
                description='Serial device connected to micon_driver_fd',
            ),
            DeclareLaunchArgument('baud', default_value='115200'),
            Node(
                package='micon_driver_fd',
                executable='serial_writer',
                name='serial_writer',
                parameters=[
                    {
                        'serial_port': serial_port,
                        'baud': baud,
                        'command_topic': '/thruster_command',
                        'use_sim_time': False,
                    }
                ],
                output='screen',
            ),
            Node(
                package='simple_manual',
                executable='thruster_id_test',
                name='thruster_id_test',
                output='screen',
                parameters=[
                    {
                        'thrust_value': 1.0,
                    }
                ],
            )
        ]
    )
