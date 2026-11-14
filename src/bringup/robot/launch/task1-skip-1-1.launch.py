"""Real-vessel two-point smoke test from Task1 GPS point 1 to point 2.

The waypoint route contains only the configured start and goal poses. Hardware
and Nav2 arguments are identical to ``task1.launch.py``.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    forwarded_arguments = {
        'role': LaunchConfiguration('role'),
        'start_role_bringup': LaunchConfiguration('start_role_bringup'),
        'start_legacy_task_nodes': LaunchConfiguration('start_legacy_task_nodes'),
        'serial_port': LaunchConfiguration('serial_port'),
        'baud': LaunchConfiguration('baud'),
        'um982_port': LaunchConfiguration('um982_port'),
        'enable_nav2_diagnostics': LaunchConfiguration('enable_nav2_diagnostics'),
        'lidar_start_delay': LaunchConfiguration('lidar_start_delay'),
        'perception_start_delay': LaunchConfiguration('perception_start_delay'),
        'nav2_start_delay': LaunchConfiguration('nav2_start_delay'),
        'waypoint_start_delay': LaunchConfiguration('waypoint_start_delay'),
        'task_type': 'task1_skip_1_1',
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_role_bringup', default_value='false',
            description=(
                'Compatibility only: include role hardware through task1.launch.py. '
                'Keep false when persistent role bringup is already running.'
            ),
        ),
        DeclareLaunchArgument(
            'start_legacy_task_nodes', default_value='false',
            description=(
                'Compatibility only: start task1 legacy Nav2/waypoints. '
                'Use /mission/run_task for normal operation.'
            ),
        ),
        DeclareLaunchArgument('role', default_value='minipc', choices=['minipc', 'standalone']),
        DeclareLaunchArgument(
            'serial_port',
            default_value=(
                '/dev/serial/by-id/'
                'usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_'
                'c82421728a9aef118808b29061ce3355-if00-port0'
            ),
        ),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument(
            'um982_port', default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
        ),
        DeclareLaunchArgument('enable_nav2_diagnostics', default_value='true'),
        DeclareLaunchArgument('lidar_start_delay', default_value='18.0'),
        DeclareLaunchArgument('perception_start_delay', default_value='30.0'),
        DeclareLaunchArgument('nav2_start_delay', default_value='35.0'),
        DeclareLaunchArgument('waypoint_start_delay', default_value='45.0'),
        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([FindPackageShare('robot'), 'launch', 'task1.launch.py'])
                    ),
                    launch_arguments=forwarded_arguments.items(),
                )
            ],
        ),
    ])
