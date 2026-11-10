"""Real-vessel Task 1 bringup: role-selected hardware, Nav2, and task waypoints."""

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
    bringup_args = {
        'serial_port': LaunchConfiguration('serial_port'),
        'baud': LaunchConfiguration('baud'),
        'um982_port': LaunchConfiguration('um982_port'),
        'enable_nav2': 'false',
        'enable_mission_manager': 'false',
        'enable_control_manager': 'false',
        'active_nav2_profile': 'task1',
    }

    minipc_role = _include(
        'robot', 'minipc_bringup.launch.py', bringup_args, condition=_role_is('minipc')
    )
    standalone_role = _include(
        'robot',
        'standalone_bringup.launch.py',
        # Only the standalone role owns local sensors, so the staged sensor
        # startup delays are forwarded here and not to minipc_bringup.
        dict(
            bringup_args,
            lidar_start_delay=LaunchConfiguration('lidar_start_delay'),
            camera_start_delay=LaunchConfiguration('perception_start_delay'),
        ),
        condition=_role_is('standalone'),
    )
    nav2 = _include(
        'robot',
        'nav2.launch.py',
        {
            'enable_diagnostics': LaunchConfiguration('enable_nav2_diagnostics'),
            # Task 1 currently navigates from UM982 odometry without LiDAR-based
            # collision monitoring. Keep the Nav2 cmd_vel relay alive, but disable
            # its LiDAR source and collision actions.
            'enable_collision_monitor': 'false',
        },
    )
    waypoints = _include(
        'waypoint_publisher',
        'waypoint_publisher.launch.py',
        {
            'task_type': LaunchConfiguration('task_type'),
            # GNSS projection, buoy detections and virtual walls all use the
            # odom -> base_link tree.
            'frame_id': 'odom',
            'publish_rate_hz': '2.0',
        },
        condition=IfCondition(LaunchConfiguration('start_legacy_task_nodes')),
    )
    cardinal_walls = Node(
        package='buoy_obstacle_publisher',
        executable='cardinal_wall_publisher',
        name='cardinal_wall_publisher',
        output='screen',
        parameters=[{
            'detection_topic': '/buoy_detections_3d',
            'output_topic': '/virtual_obstacles',
            'map_frame': 'odom',
            # UM982 dual-antenna heading is referenced to true north. The
            # wall node maps it into the navigation map frame per detection.
            'true_north_heading_topic': '/sensor/vehicle_gnss/compass/raw',
            # waypoint_publisher latches this true only after competition
            # waypoint GPS3 is passed.
            'wall_enable_topic': '/task1/cardinal_wall_enable',
            # GPS3->4 map heading is calculated by waypoint_publisher from
            # the configured (possibly geodetic) route points.
            'retire_passed_cardinal_walls_from_base_pose': True,
            'retirement_margin_m': 1.0,
            'retirement_heading_topic': '/task1/gps3_to_gps4_heading',
        }],
        condition=IfCondition(LaunchConfiguration('start_legacy_task_nodes')),
    )

    return LaunchDescription([
        LogInfo(msg=(
            'DEPRECATED: task1.launch.py owns a legacy comparison graph. '
            'Use minipc_bringup.launch.py followed by /mission/run_task for operation.'
        )),
        DeclareLaunchArgument(
            'start_role_bringup',
            default_value='false',
            description=(
                'Compatibility only: include the selected role bringup. Keep false '
                'when jetson_bringup/minipc_bringup is already persistent.'
            ),
        ),
        DeclareLaunchArgument(
            'start_legacy_task_nodes',
            default_value='false',
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
        DeclareLaunchArgument(
            'task_type',
            default_value='task1',
            choices=['task1', 'task1_skip_1_1'],
            description=(
                'task1: full course; task1_skip_1_1: omit the Task1-1 '
                'maneuvering section and run the GPS point 1-to-2 smoke test.'
            ),
        ),
        DeclareLaunchArgument(
            'lidar_start_delay',
            default_value='18.0',
            description='Seconds to wait before starting the Livox LiDAR.',
        ),
        DeclareLaunchArgument(
            'perception_start_delay',
            default_value='30.0',
            description='Seconds to wait before starting the ZED 2i / perception '
                        'components. Only used when role:=standalone.',
        ),
        DeclareLaunchArgument('nav2_start_delay', default_value='35.0'),
        DeclareLaunchArgument('waypoint_start_delay', default_value='45.0'),
        minipc_role,
        standalone_role,
        cardinal_walls,
        TimerAction(
            period=LaunchConfiguration('nav2_start_delay'), actions=[nav2],
            condition=IfCondition(LaunchConfiguration('start_legacy_task_nodes')),
        ),
        TimerAction(
            period=LaunchConfiguration('waypoint_start_delay'), actions=[waypoints],
            condition=IfCondition(LaunchConfiguration('start_legacy_task_nodes')),
        ),
    ])
