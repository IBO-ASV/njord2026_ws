"""Real-vessel Task 2 bringup: role-selected hardware, Nav2, and task waypoints."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _include(package, launch_file, arguments=None, condition=None):
    """Include a launch file in its own argument scope."""
    return GroupAction(
        scoped=True,
        condition=condition,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare(package), 'launch', launch_file])
                ),
                launch_arguments=(arguments or {}).items(),
            )
        ],
    )


def _role_is(name):
    return IfCondition(PythonExpression([
        "'", LaunchConfiguration('start_role_bringup'), "' == 'true' and '",
        LaunchConfiguration('role'), "' == '", name, "'",
    ]))


def generate_launch_description():
    robot_share = get_package_share_directory('robot')
    bringup_args = {
        'serial_port': LaunchConfiguration('serial_port'),
        'baud': LaunchConfiguration('baud'),
        'um982_port': LaunchConfiguration('um982_port'),
        'enable_nav2': 'false',
        'enable_mission_manager': 'false',
        'enable_control_manager': 'false',
        'active_nav2_profile': 'task2',
    }
    minipc_role = _include(
        'robot', 'minipc_bringup.launch.py', bringup_args, condition=_role_is('minipc')
    )
    standalone_role = _include(
        'robot', 'standalone_bringup.launch.py', bringup_args, condition=_role_is('standalone')
    )
    nav2 = _include(
        'robot',
        'navigation_launch_task2.py',
        {
            'params_file': os.path.join(robot_share, 'config', 'nav2_params_task2_humble.yaml'),
        },
    )
    waypoints = _include(
        'waypoint_publisher',
        'waypoint_publisher.launch.py',
        {'task_type': 'task2', 'frame_id': 'map', 'publish_rate_hz': '2.0'},
        condition=IfCondition(LaunchConfiguration('start_legacy_task_nodes')),
    )

    return LaunchDescription([
        LogInfo(msg=(
            'DEPRECATED: task2.launch.py owns a legacy comparison graph. '
            'Use minipc_bringup.launch.py followed by /mission/run_task for operation.'
        )),
        DeclareLaunchArgument(
            'start_role_bringup', default_value='false',
            description=(
                'Compatibility only: include the selected role bringup. Keep false '
                'when jetson_bringup/minipc_bringup is already persistent.'
            ),
        ),
        DeclareLaunchArgument(
            'start_legacy_task_nodes', default_value='false',
            description=(
                'Compatibility only: start the legacy Nav2 and waypoint graph. '
                'Use /mission/run_task for normal operation.'
            ),
        ),
        DeclareLaunchArgument(
            'role',
            default_value='minipc',
            choices=['minipc', 'standalone'],
            description='minipc: 2-machine split (Jetson hosts GLIM/LiDAR/ZED separately). '
            'standalone: single-Jetson regression bringup.',
        ),
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
            'um982_port',
            default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
        ),
        DeclareLaunchArgument('enable_nav2_diagnostics', default_value='true'),
        DeclareLaunchArgument('nav2_start_delay', default_value='35.0'),
        DeclareLaunchArgument('waypoint_start_delay', default_value='45.0'),
        minipc_role,
        standalone_role,
        TimerAction(
            period=LaunchConfiguration('nav2_start_delay'), actions=[nav2],
            condition=IfCondition(LaunchConfiguration('start_legacy_task_nodes')),
        ),
        TimerAction(
            period=LaunchConfiguration('waypoint_start_delay'), actions=[waypoints],
            condition=IfCondition(LaunchConfiguration('start_legacy_task_nodes')),
        ),
    ])
