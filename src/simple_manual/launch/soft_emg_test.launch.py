"""Bring up only joy_converter and serial_writer to test software EMG in isolation.

Does not start thruster_driver, localization, or perception. Start `joy_node`
separately (e.g. on the shore PC, see README.md) so `/joy` reaches this host.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    baud = LaunchConfiguration('baud')

    joy_converter = Node(
        package='simple_manual',
        executable='joy_converter_node',
        name='joy_converter',
        output='screen',
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare('simple_manual'), 'config', 'joy_converter.yaml']
            )
        ],
    )
    serial_writer = Node(
        package='micon_driver_fd',
        executable='serial_writer',
        name='serial_writer',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'baud': baud,
            'command_topic': '/thruster_command',
            'use_sim_time': False,
        }],
    )

    return LaunchDescription([
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
        joy_converter,
        serial_writer,
    ])
