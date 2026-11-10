"""Manual-control bringup with one intra-process LiDAR/ZED perception container."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


LIDAR_CONFIGS = {
    'mid360': 'MID360_config.json',
    'mid360s': 'MID360S_jetson_config.json',
}


def _as_bool(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ('true', '1', 'yes', 'on')


def _sensor_container(context, *_args, **_kwargs):
    """Start the sensor-driver components before perception is loaded."""
    lidar_model = LaunchConfiguration('lidar_model').perform(context)
    try:
        lidar_config = LIDAR_CONFIGS[lidar_model]
    except KeyError as error:
        raise RuntimeError(f'Unsupported lidar_model: {lidar_model}') from error

    components = [
        ComposableNode(
            package='livox_ros_driver2',
            plugin='livox_ros::DriverNode',
            name='livox_lidar_publisher',
            parameters=[
                {
                    'xfer_format': 0,
                    'multi_topic': 0,
                    'data_src': 0,
                    'publish_freq': 10.0,
                    'output_data_type': 0,
                    'frame_id': 'livox_frame',
                    'lvx_file_path': '/home/livox/livox_test.lvx',
                    'user_config_path': PathJoinSubstitution(
                        [FindPackageShare('robot'), 'config', 'livox', lidar_config]
                    ),
                    'cmdline_input_bd_code': 'livox0000000001',
                }
            ],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
        ComposableNode(
            package='zed2i_driver',
            plugin='zed2i_driver::SdkNode',
            name='zed2i',
            namespace='/zed2i',
            parameters=[
                PathJoinSubstitution(
                    [FindPackageShare('zed2i_driver'), 'config', 'zed2i_jetson_orin_nano.yaml']
                ),
                {
                    'enable_gpu_perception': _as_bool(context, 'enable_gpu_perception'),
                    'engine_path': LaunchConfiguration('engine_path'),
                    'camera_resolution': LaunchConfiguration('camera_resolution'),
                    'framerate': ParameterValue(
                        LaunchConfiguration('camera_framerate'), value_type=int
                    ),
                    'enable_ground_video': False,
                    'ground_video_host': '',
                    'ground_video_port': 5600,
                    'ground_video_fps': 5.0,
                },
            ],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
    ]

    return [
        ComposableNodeContainer(
            package='rclcpp_components',
            executable='component_container_mt',
            name='sensor_perception_container',
            namespace='/',
            output='screen',
            composable_node_descriptions=components,
        )
    ]


def _load_perception_components(context, *_args, **_kwargs):
    """Load optional perception components into the running sensor container."""
    components = []

    if _as_bool(context, 'enable_pcl_buoy_detection'):
        components.append(
            ComposableNode(
                package='pcl_det',
                plugin='pcl_det::PclBuoyDetectionNode',
                name='pcl_bouy_det_node',
                parameters=[
                    {
                        'input_topic': '/livox/lidar',
                        'roi_topic': '/buoy_roi',
                        'output_topic': '/buoy_detections',
                        'frame_id': 'base_link',
                    }
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        )

    if not components:
        return []

    return [
        LoadComposableNodes(
            target_container='/sensor_perception_container',
            composable_node_descriptions=components,
        )
    ]


def generate_launch_description():
    robot_share = FindPackageShare('robot')
    default_thruster_config = os.path.join(
        get_package_share_directory('thruster_driver'), 'config', 'config.yaml'
    )
    default_thruster_urdf = os.path.join(
        get_package_share_directory('robot'), 'urdf', 'robot.urdf_modified.urdf'
    )
    default_engine = os.path.join(
        get_package_share_directory('robot'), 'config', 'yolo_model', 'best.engine'
    )

    robot_description = Command([
        FindExecutable(name='xacro'),
        ' ',
        PathJoinSubstitution([robot_share, 'urdf', 'robot.urdf.xacro']),
    ])

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
    command_arbiter = Node(
        package='simple_manual',
        executable='command_arbiter_node',
        name='command_arbiter',
        output='screen',
        parameters=[{'auto_topic': LaunchConfiguration('auto_topic')}],
    )
    micon_driver = Node(
        package='micon_driver_fd',
        executable='serial_writer',
        name='serial_writer',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud': LaunchConfiguration('baud'),
            'command_topic': '/thruster_command',
            'bms_topic': 'micon/bms_cells',
            'use_sim_time': False,
        }],
    )
    bms = Node(
        package='bms',
        executable='bms_node',
        name='bms_node',
        output='screen',
        parameters=[PathJoinSubstitution([FindPackageShare('bms'), 'config', 'config.yaml'])],
    )
    heading_arrow = Node(
        package='tf_frame_arrow_publisher',
        executable='arrow_publisher',
        name='nav_arrow_publisher',
        output='screen',
    )
    actual_route = Node(
        package='tf_frame_arrow_publisher',
        executable='full_path_publisher',
        name='actual_route_publisher',
        output='screen',
        parameters=[{
            'marker_topic': '/actual_path_marker',
            'parent_frame': 'odom',
            'child_frame': 'base_link',
        }],
    )
    planned_route = Node(
        package='tf_frame_arrow_publisher',
        executable='planned_path_marker_publisher',
        name='planned_route_publisher',
        output='screen',
        parameters=[{
            'path_topic': '/plan_smoothed',
            'marker_topic': '/planned_path_marker',
        }],
    )
    ground_speed = Node(
        package='tf_frame_arrow_publisher',
        executable='ground_speed_publisher',
        name='ground_speed_publisher',
        output='screen',
        parameters=[{'odometry_topic': '/odometry/feedback'}],
    )
    um982_driver = Node(
        package='um982_driver',
        executable='um982_driver_node',
        name='um982_driver',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'uart_or_tcp': 'uart',
            'GNSS_SerialPort': LaunchConfiguration('um982_port'),
            'GNSS_Baudrate': 115200,
            'tcp_ip': '192.168.0.126',
            'tcp_port': 23,
            'FIX_FREQ': 20,
            'HEADING_FREQ': 20,
            'GNSS_RTK_Enable': False,
            'NTRIP_Server': 'ntrip.ales-corp.co.jp',
            'NTRIP_Port': 2101,
            'NTRIP_Mountpoint': 'RTCM32MSM7',
            'NTRIP_Username': '',
            'NTRIP_Password': '',
            'Heading_FrameID': 'odom',
            'log_file_name': '',
            'publish_feedback_odometry': LaunchConfiguration('enable_um982_velocity_feedback'),
        }],
    )
    # Drogger is intentionally disabled. Global localization uses UM982 only.
    # drogger_rzs = Node(
    #     package='drogger_wired_flex',
    #     executable='drogger_wired_flex_node',
    #     name='drogger_wired_flex',
    #     output='screen',
    #     parameters=[
    #         PathJoinSubstitution(
    #             [FindPackageShare('drogger_wired_flex'), 'config', 'params_rzs_d01_usb.yaml']
    #         ),
    #         {
    #             'serial_port': LaunchConfiguration('drogger_rzs_port'),
    #             'serial_baudrate': ParameterValue(115200, value_type=int),
    #             'fix_topic': '/gnss/fix',
    #         },
    #     ],
    # )
    thruster_driver = Node(
        package='thruster_driver',
        executable='thruster_driver_node',
        name='thruster_driver_node',
        output='screen',
        parameters=[
            LaunchConfiguration('thruster_config_file'),
            {
                'robot_description': ParameterValue(
                    Command(['cat ', LaunchConfiguration('thruster_robot_description_file')]),
                    value_type=str,
                ),
            },
            {
                # Manual control is open-loop: do not require odometry feedback.
                'control.use_velocity_feedback': False,
            },
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str)}],
    )
    um982_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='um982_static_tf_pub',
        output='screen',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'base_link', '--child-frame-id', 'um982_link',
        ],
    )
    local_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_local',
        output='screen',
        parameters=[
            PathJoinSubstitution([robot_share, 'config', 'ekf_local.yaml']),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[('odometry/filtered', 'odometry/filtered/local')],
    )
    global_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_global',
        output='screen',
        parameters=[
            PathJoinSubstitution([robot_share, 'config', 'ekf_global.yaml']),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[('odometry/filtered', 'odometry/filtered/global')],
    )
    navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        arguments=['--ros-args', '--log-level', 'ERROR'],
        parameters=[{
            'world_frame': 'map',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'frequency': 10.0, 'magnetic_declination_radians': 0.0, 'yaw_offset': 0.0,
            'zero_altitude': True, 'broadcast_utm_transform': True, 'publish_filtered_gps': True,
            'use_odometry_yaw': True, 'wait_for_datum': False,
        }],
        remappings=[
            ('gps/fix', '/sensor/vehicle_gnss/fix/raw'),
            ('odometry/filtered', 'odometry/filtered/local'),
            ('odometry/gps', '/odometry/gps/um982'),
        ],
    )
    um982_feedback = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('um982_feedback_filter'), 'launch', 'um982_feedback.launch.py'
            ])
        ),
        launch_arguments={
            'feedback_mode': LaunchConfiguration('um982_feedback_mode'),
            'raw_topic': '/odometry/feedback',
            'output_topic': '/odometry/filtered/local',
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_um982_velocity_feedback')),
    )

    diagnostics = ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container_mt',
        name='localization_diagnostic_monitor_container',
        namespace='',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_diagnostics')),
        composable_node_descriptions=[
            ComposableNode(
                package='diagnostic_monitors',
                plugin='njord::diagnostic_monitors::TopicHeartbeatMonitor',
                name=f'{name}_heartbeat',
                parameters=[{
                    'monitor_name': name,
                    'topic': topic,
                    'topic_type': topic_type,
                    'mode': 'required_frequency',
                    'expected_frequency': expected,
                    'minimum_frequency': minimum,
                    'timeout': timeout,
                    'stale_timeout': stale,
                    'window_size': 50,
                    'qos_depth': 10,
                    'qos_reliability': 'best_effort',
                }],
            )
            for name, topic, topic_type, expected, minimum, timeout, stale in [
                ('livox_imu', '/livox/imu', 'sensor_msgs/msg/Imu', 200.0, 50.0, 0.5, 2.0),
                (
                    'gps_fix', '/sensor/vehicle_gnss/fix/raw',
                    'sensor_msgs/msg/NavSatFix', 20.0, 10.0, 1.0, 3.0,
                ),
                (
                    'local_filtered_odom', '/odometry/filtered/local',
                    'nav_msgs/msg/Odometry', 30.0, 5.0, 1.0, 4.0,
                ),
                (
                    'global_filtered_odom', '/odometry/filtered/global',
                    'nav_msgs/msg/Odometry', 30.0, 5.0, 1.0, 4.0,
                ),
            ]
        ],
    )

    return LaunchDescription([
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
            'use_sim_time', default_value='false',
            description='Use the /clock topic (set true when replaying a rosbag).',
        ),
        DeclareLaunchArgument('auto_topic', default_value='/cmd_vel_nav'),
        DeclareLaunchArgument(
            'um982_port',
            default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
        ),
        DeclareLaunchArgument(
            'enable_um982_velocity_feedback', default_value='true',
            description=(
                'Use UM982 dual-antenna heading and GNSS position differences as '
                'thruster velocity feedback. Requires an outdoor GNSS fix.'
            ),
        ),
        DeclareLaunchArgument(
            'um982_feedback_mode', default_value='ekf', choices=['window', 'ekf'],
            description='UM982-only feedback filter: time-window regression or independent EKF.',
        ),
        # Drogger is intentionally disabled. Keep this argument commented out
        # until the driver is restored.
        # DeclareLaunchArgument(
        #     'drogger_rzs_port',
        #     default_value=(
        #         '/dev/serial/by-id/'
        #         'usb-Prolific_Technology_Inc._USB-Serial_Controller_'
        #         'ACCQg146B12-if00-port0'
        #     ),
        # ),
        DeclareLaunchArgument(
            'lidar_model', default_value='mid360s', choices=['mid360', 'mid360s']
        ),
        DeclareLaunchArgument(
            'enable_pcl_buoy_detection', default_value='false',
            description='Deprecated point-cloud-only detector; GPU camera/LiDAR fusion is preferred.',
        ),
        DeclareLaunchArgument('enable_gpu_perception', default_value='true'),
        DeclareLaunchArgument('engine_path', default_value=default_engine),
        DeclareLaunchArgument('camera_resolution', default_value='HD720'),
        DeclareLaunchArgument('camera_framerate', default_value='15'),
        DeclareLaunchArgument('lidar_start_delay', default_value='18.0'),
        DeclareLaunchArgument(
            'perception_start_delay',
            default_value='20.0',
            description='Seconds to wait before loading buoy detection components.',
        ),
        DeclareLaunchArgument(
            'localization_start_delay',
            default_value='30.0',
            description='Seconds to wait before starting localization and its diagnostics.',
        ),
        DeclareLaunchArgument('thruster_config_file', default_value=default_thruster_config),
        DeclareLaunchArgument(
            'thruster_robot_description_file', default_value=default_thruster_urdf
        ),
        DeclareLaunchArgument('enable_diagnostics', default_value='true'),

        # diagnostics,

        joy_converter,
        command_arbiter,
        micon_driver,
        bms,
        heading_arrow,
        actual_route,
        planned_route,
        ground_speed,
        # um982_driver,
        # um982_feedback,
        # drogger_rzs,
        thruster_driver,
        # robot_state_publisher,
        # um982_static_tf,
        TimerAction(
            period=LaunchConfiguration('lidar_start_delay'),
            actions=[OpaqueFunction(function=_sensor_container)],
        ),
        TimerAction(
            period=LaunchConfiguration('perception_start_delay'),
            actions=[OpaqueFunction(function=_load_perception_components)],
        ),
        # TimerAction(
        #     period=LaunchConfiguration('localization_start_delay'),
        #     actions=[global_ekf, navsat_transform],
        # ),
        # TimerAction(
        #     period=LaunchConfiguration('localization_start_delay'),
        #     actions=[local_ekf],
        #     # Both UM982 feedback and the local EKF publish
        #     # /odometry/filtered/local. Do not run the EKF path while this
        #     # direct feedback path owns that topic.
        #     condition=UnlessCondition(LaunchConfiguration('enable_um982_velocity_feedback')),
        # ),
    ])
