import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('drogger_wired_flex')
    default_params_file = os.path.join(pkg_share, 'config', 'params_rzs_d01_usb.yaml')
    params_file = LaunchConfiguration('config_file')
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    fix_topic = LaunchConfiguration('fix_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_params_file,
            description='Path to YAML parameter file',
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value=(
                '/dev/serial/by-id/'
                'usb-Prolific_Technology_Inc._USB-Serial_Controller_'
                'ACCQg146B12-if00-port0'
            ),
            description='RZS D01 serial device (prefer a stable by-id path)',
        ),
        DeclareLaunchArgument('serial_baudrate', default_value='115200'),
        DeclareLaunchArgument('fix_topic', default_value='/gnss/fix'),
        Node(
            package='drogger_wired_flex',
            executable='drogger_wired_flex_node',
            name='drogger_wired_flex',
            output='screen',
            parameters=[
                params_file,
                {
                    'serial_port': serial_port,
                    'serial_baudrate': ParameterValue(
                        serial_baudrate, value_type=int
                    ),
                    'fix_topic': fix_topic,
                },
            ],
        )
    ])
