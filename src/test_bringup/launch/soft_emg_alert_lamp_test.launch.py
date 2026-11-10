"""
Run the soft-EMG path with the alert-lamp manager and driver.

Keep propulsion power off or the vessel mechanically restrained.  This launch
does not start localization, autonomy, or heartbeat publishers, so alert_lamp
will remain in its fail-safe red state until those inputs are supplied.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    baud = LaunchConfiguration('baud')

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value=(
                '/dev/serial/by-id/'
                'usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_'
                'c82421728a9aef118808b29061ce3355-if00-port0'
            ),
            description='ESP32 serial device used by micon_driver_fd.',
        ),
        DeclareLaunchArgument('baud', default_value='115200'),
        Node(
            package='simple_manual',
            executable='joy_converter_node',
            name='joy_converter',
            output='screen',
            parameters=[
                PathJoinSubstitution(
                    [FindPackageShare('simple_manual'), 'config', 'joy_converter.yaml']
                )
            ],
            # Keep alert_lamp_driver as the only publisher to the physical lamp topics.
            remappings=[
                ('/green', '/soft_emg_test/manual_green'),
                ('/yellow', '/soft_emg_test/manual_yellow'),
                ('/red', '/soft_emg_test/manual_red'),
            ],
        ),
        Node(
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
        ),
        Node(
            package='alert_lamp',
            executable='alert_lamp_manager_node',
            name='alert_lamp_manager_node',
            output='screen',
            parameters=[
                PathJoinSubstitution(
                    [FindPackageShare('alert_lamp'), 'config', 'alert_lamp.yaml']
                )
            ],
        ),
        Node(
            package='alert_lamp',
            executable='alert_lamp_driver_node',
            name='alert_lamp_driver_node',
            output='screen',
            parameters=[
                PathJoinSubstitution(
                    [FindPackageShare('alert_lamp'), 'config', 'alert_lamp.yaml']
                )
            ],
        ),
    ])
