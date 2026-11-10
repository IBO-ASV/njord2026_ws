import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_task3_sim  = get_package_share_directory("task3_sim")
    pkg_robot      = get_package_share_directory("robot")
    pkg_dutyed     = get_package_share_directory("dutyed_tf_pub_with_disturbance")
    pkg_waypoint   = get_package_share_directory("waypoint_publisher")
    pkg_thruster   = get_package_share_directory("thruster_driver")
    robot_description_file = os.path.join(pkg_robot, 'urdf', 'robot.urdf.xacro')
    robot_description = Command(['xacro ', robot_description_file])

    # ── Launch arguments ──────────────────────────────────────────────────────
    # Startup timing rationale:
    #   t=0.0  SENSOR layer: sim_dynamics, sensor_noise (including UM982),
    #            orchestrator and EKFs
    #   t=5.0  NAV2 layer:   Nav2 bringup (needs SimNode TF + filtered odometry)
    #   t=8.0  GOAL layer:   waypoint_publisher (needs NavigateThroughPoses action server)
    #
    # Why 5s for Nav2?
    #   UM982 simulator starts on the first /odom truth message and publishes
    #   /odometry/gps/um982. The global EKF then starts publishing global odometry.
    #   But lifecycle_manager configure+activate for all Nav2 nodes takes 2-5s.
    #   Total margin: 5s should be safe for sim where GPS is synthetic and instant.
    #
    # Why 8s for waypoint_publisher?
    #   NavigateThroughPoses action server is the LAST thing to come up in Nav2.
    #   bt_navigator depends on planner + controller being active first.
    #   3s extra beyond Nav2 start is sufficient.
    driver_delay_arg = DeclareLaunchArgument(
        'driver_delay', default_value='0.0',
        description='Delay before launching sensor/EKF layer')
    nav2_delay_arg = DeclareLaunchArgument(
        'nav2_delay', default_value='5.0',
        description='Delay before launching Nav2 (needs SimNode TF and filtered odometry)')
    goal_delay_arg = DeclareLaunchArgument(
        'goal_delay', default_value='12.0',
        description='Delay before launching waypoint_publisher (needs Nav2 action servers up)')
    task_type_arg = DeclareLaunchArgument(
        'task_type', default_value='task3_2',
        description='Task type: task3_1 or task3_2')
    full_sequence_arg = DeclareLaunchArgument(
        'run_full_sequence', default_value='true',
        description='For task3_1, continue through task3_2 and finish at GPS10')
    enable_diagnostics_arg = DeclareLaunchArgument(
        'enable_diagnostics', default_value='true',
        description='Launch generic topic heartbeat diagnostics for Task3 simulation')

    driver_delay = LaunchConfiguration('driver_delay')
    nav2_delay   = LaunchConfiguration('nav2_delay')
    goal_delay   = LaunchConfiguration('goal_delay')
    task_type    = LaunchConfiguration('task_type')
    run_full_sequence = LaunchConfiguration('run_full_sequence')
    enable_diagnostics = LaunchConfiguration('enable_diagnostics')

    # ── SENSOR / PHYSICS LAYER (t=0) ─────────────────────────────────────────
    # SimNode is the sole Task3 simulation TF authority.
    sim_dynamics = Node(
        package="dutyed_tf_pub_with_disturbance",
        executable="dutyed_tf_pub_with_disturbance_node",
        name="dutyed_tf_pub_with_disturbance_node",
        parameters=[
            os.path.join(pkg_dutyed, "config", "node_config.yaml"),
            {
                "publish_tf": True,
                # Simulation uses geometric body-frame forces directly; do
                # not apply the real vessel's port-side wiring correction.
                "thruster_force_sign": [1.0, 1.0, 1.0, 1.0],
            }
        ],
        output="screen"
    )

    # Sensor noise simulator: /odom → /livox/imu, legacy GPS/compass, and
    # /odometry/gps/um982 for the global EKF.
    pkg_sensor_noise = get_package_share_directory("sensor_sim_with_noise")
    sensor_noise_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sensor_noise, "launch", "sensor_noise.launch.py")),
        launch_arguments={
            # The Foxglove GNSS Map Telemetry panel subscribes to this
            # real-vessel GNSS contract.
            'fix_topic': '/sensor/vehicle_gnss/fix/raw',
            # Keep Task3's localization interface aligned with the vessel.
            'imu_topic': '/livox/imu',
        }.items(),
    )

    # Task3 orchestrator: buoys (b31_* + b32_*), dock geometry, PointCloud2, TFs
    config = os.path.join(pkg_task3_sim, "config", "task3_params.yaml")
    task3_orchestrator = Node(
        package="task3_sim",
        executable="task3_orchestrator",
        name="task3_orchestrator",
        parameters=[
            config,
            {
                "task_type": task_type,
                "run_full_sequence": ParameterValue(run_full_sequence, value_type=bool),
            },
        ],
        output="screen",
    )

    # Simulated BMS and operating-mode telemetry for the Foxglove overview
    # indicators. These mirror the real-vessel GUI topic contracts.
    gui_status_dummy = Node(
        package="task3_sim",
        executable="task3_gui_status_dummy",
        name="task3_gui_status_dummy",
        parameters=[config],
        output="screen",
    )
    foxglove_logger = Node(
        package="foxglove_logger",
        executable="foxglove_logger_node",
        name="foxglove_logger",
        parameters=[{
            'odometry_topic': '/odometry/filtered/global',
            'plan_topic': '/plan',
            'fix_topic': '/sensor/vehicle_gnss/fix/raw',
        }],
        output='screen',
    )

    heading_arrow = Node(
        package="tf_frame_arrow_publisher",
        executable="arrow_publisher",
        name="nav_arrow_publisher",
        output="screen",
    )
    actual_route = Node(
        package="tf_frame_arrow_publisher",
        executable="full_path_publisher",
        name="actual_route_publisher",
        output="screen",
        parameters=[{
            "marker_topic": "/actual_path_marker",
            "parent_frame": "odom",
            "child_frame": "base_link",
        }],
    )

    # Publish simulated SOG for the Foxglove GNSS Map Telemetry panel.  In
    # simulation, /odom is the truth odometry and already carries ENU speed.
    ground_speed = Node(
        package="tf_frame_arrow_publisher",
        executable="ground_speed_publisher",
        name="ground_speed_publisher",
        parameters=[{'odometry_topic': '/odom'}],
        output="screen",
    )

    # Thruster driver: cmd_vel -> /thruster_command (Int16MultiArray)
    thruster_driver_node = Node(
        package="thruster_driver",
        executable="thruster_driver_node",
        name="thruster_driver_node",
        parameters=[
            os.path.join(pkg_thruster, "config", "config.yaml"),
            {
                "robot_description": ParameterValue(robot_description, value_type=str),
                "transport_mode": "sim",
                "control.dob.enable": False,
                "allocation.wrench_sign": [1.0, 1.0, 1.0],
                "thrusters.reverse": [False, False, False, False],
            },
        ],
        output="screen",
    )

    # Match the vessel control contract used by the shared Nav2 launch:
    # collision_monitor publishes /cmd_vel_nav and twist_mux is the only
    # publisher of /cmd_vel consumed by the thruster driver.
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        parameters=[os.path.join(pkg_robot, "config", "twist_mux.yaml")],
        remappings=[("cmd_vel_out", "/cmd_vel")],
        output="screen",
    )

    # Robot state publisher: URDF → base_link→{imu_link, livox_frame, gnss_link, …}
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
            'use_sim_time': False
        }]
    )

    # Task3 localization: simulated UM982 position + /livox/imu orientation
    # and angular velocity. /odom remains simulation truth only and is not
    # fused into the navigation estimate.
    global_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_global',
        output='screen',
        parameters=[
            os.path.join(pkg_task3_sim, 'config', 'task3_ekf_global.yaml'),
            {'publish_tf': False},
        ],
        remappings=[('odometry/filtered', 'odometry/filtered/global')]
    )

    # Field boundary costmap: latched — outside 40m×40m square = LETHAL
    field_boundary_node = Node(
        package='buoy_obstacle_publisher',
        executable='field_boundary_publisher',
        name='field_boundary_publisher',
        parameters=[{
            'map_frame': 'map',
            'resolution': 0.2,
            'map_size_m': 80.0,
            # GPS points fit in x=[-18, 18], y=[-11, 10]. The x margin also
            # encloses the fixed docks, whose rear walls reach x=+/-21 m.
            'field_size_x_m': 48.0,
            'field_size_y_m': 28.0,
            'field_center_x': 0.0,
            'field_center_y': 0.0,
            'boundary_cost': 100,
            'include_task3_docks': True,
            'dock_wall_thickness_m': 0.3,
        }],
        output='screen',
    )

    # Dynamic lethal obstacles around the Task3 buoy TF frames.  This also
    # satisfies the task3 diagnostic monitor for /buoy_costmap.
    buoy_costmap_node = Node(
        package='buoy_obstacle_publisher',
        executable='buoy_obstacle_publisher',
        name='buoy_obstacle_publisher',
        output='screen',
    )

    sensor_layer_timer = TimerAction(
        period=driver_delay,
        actions=[
            sim_dynamics,
            sensor_noise_launch,
            task3_orchestrator,
            gui_status_dummy,
            foxglove_logger,
            ground_speed,
            thruster_driver_node,
            twist_mux,
            robot_state_pub_node,
            global_ekf_node,
            field_boundary_node,
            buoy_costmap_node,
        ]
    )

    # ── NAV2 LAYER (t=5s) ────────────────────────────────────────────────────
    # Start Nav2 after SimNode TF and both EKF odometry topics are available.
    # lifecycle_manager will configure+activate: controller, smoother, planner,
    # behavior, bt_navigator, waypoint_follower, velocity_smoother (~3-5s).
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot, "launch", "nav2.launch.py")),
        launch_arguments={
            'params_file': os.path.join(pkg_robot, 'config', 'nav2_params_task3_jazzy.yaml'),
            'enable_diagnostics': 'false',
        }.items()
    )

    nav2_layer_timer = TimerAction(
        period=nav2_delay,
        actions=[nav2_launch]
    )

    # ── GOAL LAYER (t=8s) ────────────────────────────────────────────────────
    # Start waypoint_publisher after Nav2 action servers are active.
    # The node itself polls for NavigateThroughPoses to be ready, so extra
    # margin here prevents spurious "server not ready" log spam.
    waypoint_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_waypoint, "launch", "waypoint_publisher.launch.py")),
        launch_arguments={
            'task_type': task_type,
            'frame_id': 'map',
            'publish_rate_hz': '2.0',
            'use_dynamic_gate_midpoints': 'true',
            'run_full_sequence': run_full_sequence,
        }.items()
    )

    goal_layer_timer = TimerAction(
        period=goal_delay,
        actions=[waypoint_publisher]
    )

    diagnostics_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot, "launch", "diagnostics.launch.py")),
        launch_arguments={'profile': 'task3'}.items(),
        condition=IfCondition(enable_diagnostics),
    )

    startup_message = LogInfo(msg='========== Task3 Sim Bringup Started ==========')

    return LaunchDescription([
        startup_message,
        heading_arrow,
        actual_route,
        driver_delay_arg,
        nav2_delay_arg,
        goal_delay_arg,
        task_type_arg,
        full_sequence_arg,
        enable_diagnostics_arg,
        sensor_layer_timer,
        nav2_layer_timer,
        goal_layer_timer,
        diagnostics_launch,
    ])
