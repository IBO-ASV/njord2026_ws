import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def include_launch(package_name, relative_path, condition, launch_arguments=None):
    package_share = get_package_share_directory(package_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_share, *relative_path)),
        condition=condition,
        launch_arguments=(launch_arguments or {}).items(),
    )


def launch_cardinal_walls(context):
    """Read course bounds from the orchestrator config, our sole source."""
    with open(LaunchConfiguration("params").perform(context), "r") as params_file:
        params = yaml.safe_load(params_file) or {}
    bounds = params.get("task1_orchestrator", {}).get("ros__parameters", {}).get(
        "course_bounds", [-5.0, 55.0, -40.0, 35.0]
    )
    orchestrator_params = params.get("task1_orchestrator", {}).get("ros__parameters", {})
    return [Node(
        package="buoy_obstacle_publisher",
        executable="cardinal_wall_publisher",
        name="cardinal_wall_publisher",
        parameters=[{
            "detection_topic": "/buoy_detections_3d",
            "output_topic": "/virtual_obstacles",
            "map_frame": "map",
            "course_bounds": bounds,
            "wall_width_m": 0.2,
            "point_spacing_m": 0.05,
            "confirmations_required": 2,
            # Keep the nearest four not-yet-passed marks in the GPS3->GPS4
            # direction; newly detected marks are re-ranked immediately.
            "max_active_wall_tracks": 4,
            # Main Task1.2 direction: GPS waypoint 3 -> 4 (west in this map).
            "course_heading_rad": 3.141592653589793,
            "retirement_course_heading_rad": 3.141592653589793,
            "retire_passed_cardinal_walls_from_base_pose": True,
            "retirement_margin_m": 1.0,
            "retirement_confirmations_required": 5,
            "return_confirmations_required": 5,
            "retirement_heading_topic": "/task1/gps3_to_gps4_heading",
            # The simulator publishes the same UM982 true-heading contract
            # used by the vessel's dual-antenna GNSS receiver.
            "true_north_heading_topic": "/sensor/vehicle_gnss/compass/raw",
            "true_north_confirmations_required": 10,
            "wall_enable_topic": "/task1/cardinal_wall_enable",
            "retirement_frontier_topic": "/sim/task1_wall_retirement_frontier",
            # Show the known simulator marks faintly before perception
            # confirms them. The wall node keeps these out of Nav2's actual
            # /virtual_obstacles output.
            "preview_buoy_positions": orchestrator_params.get("buoy_position_xy", "[]"),
            "preview_buoy_marks": orchestrator_params.get("buoy_marks", "[]"),
        }],
        output="screen",
    )]


def launch_cardinal_perception_sim(context):
    """Sim stand-in for the ZED2i+Mid-360 intra-process cardinal-marker
    pipeline. Reuses buoy_position_xy from the orchestrator config so marker
    geometry has a single source of truth."""
    with open(LaunchConfiguration("params").perform(context), "r") as params_file:
        params = yaml.safe_load(params_file) or {}
    orchestrator_params = params.get("task1_orchestrator", {}).get("ros__parameters", {})
    buoy_position_xy = orchestrator_params.get(
        "buoy_position_xy", "[[28.0, -25.0], [18.0, -25.0], [11.0, -25.0]]"
    )
    buoy_marks = orchestrator_params.get("buoy_marks", "[]")
    return [Node(
        package="task1_sim",
        executable="cardinal_perception_sim",
        name="cardinal_perception_sim",
        parameters=[{
            "odom_topic": "/odom",
            "cardinal_mark_topic": "/sim/cardinal_mark",
            "detection_topic": "/buoy_detections_3d",
            # buoy_position_xy is map-frame simulator ground truth.  Keeping
            # the output in map avoids creating duplicate wall tracks when
            # map and odom diverge during the simulated run.
            "output_frame": "map",
            "buoy_position_xy": buoy_position_xy,
            "buoy_marks": buoy_marks,
            "recognition_range_m": 100.0,
        }],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_cardinal_perception_sim")),
    )]


def launch_dynamics(context, pkg_dutyed):
    """Create the dynamics node, optionally staged just before competition WP3."""
    start_at_wp3 = LaunchConfiguration("start_at_wp3").perform(context).lower() == "true"
    initial_pose = [50.0, -20.0, -1.5707963267948966] if start_at_wp3 else [0.0, 0.0, 0.0]
    return [Node(
        package="dutyed_tf_pub_with_disturbance",
        executable="dutyed_tf_pub_with_disturbance_node",
        name="dutyed_tf_pub_with_disturbance_node",
        parameters=[
            os.path.join(pkg_dutyed, "config", "node_config.yaml"),
            {
                "publish_tf": True,
                "initial_pose_xyyaw": initial_pose,
                # Simulation uses geometric body-frame forces directly; do
                # not apply the real vessel's port-side wiring correction.
                "thruster_force_sign": [1.0, 1.0, 1.0, 1.0],
            },
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_dynamics")),
    )]


def generate_launch_description():
    pkg_share = get_package_share_directory("task1_sim")
    pkg_robot = get_package_share_directory("robot")
    pkg_dutyed = get_package_share_directory("dutyed_tf_pub_with_disturbance")
    pkg_sensor_noise = get_package_share_directory("sensor_sim_with_noise")
    pkg_thruster = get_package_share_directory("thruster_driver")

    config = os.path.join(pkg_share, "config", "task1_params.yaml")
    ros_distro = os.environ.get("ROS_DISTRO", "").lower()
    nav2_params_name = (
        "task1_nav2_params_jazzy.yaml" if ros_distro == "jazzy"
        else "task1_nav2_params_humble.yaml"
    )
    nav2_params = os.path.join(pkg_share, "config", nav2_params_name)
    nav_through_poses_bt_xml = os.path.join(
        pkg_share,
        "behavior_trees",
        "navigate_through_poses_w_replanning_and_recovery.xml",
    )
    nav_to_pose_bt_xml = os.path.join(
        pkg_robot,
        "config",
        "navigate_to_pose_w_replanning_and_recovery.xml",
    )
    robot_description_file = os.path.join(pkg_robot, "urdf", "robot.urdf_modified.urdf")
    robot_description = open(robot_description_file, "r").read()

    # Startup timing (mirrors task2/task3):
    #   t=0.0  SENSOR layer: sim_dynamics, sensor_noise (including UM982),
    #            thruster_driver, orchestrator, EKFs
    #   t=5.0  NAV2 layer:   Nav2 bringup (needs SimNode TF + filtered odometry)
    #   t=8.0  GOAL layer:   waypoint_publisher (needs NavigateThroughPoses action server)
    use_dynamics_arg = DeclareLaunchArgument("use_dynamics", default_value="true")
    use_nav2_arg = DeclareLaunchArgument("use_nav2", default_value="true")
    use_thruster_driver_arg = DeclareLaunchArgument("use_thruster_driver", default_value="true")
    use_waypoints_arg = DeclareLaunchArgument("use_waypoints", default_value="true")
    use_validator_arg = DeclareLaunchArgument("use_validator", default_value="true")
    use_sensor_noise_arg = DeclareLaunchArgument("use_sensor_noise", default_value="true")
    use_gui_dummy_publishers_arg = DeclareLaunchArgument(
        "use_gui_dummy_publishers",
        default_value="true",
        description="Publish simulated battery percentage and autonomous control status for Foxglove",
    )
    use_foxglove_bridge_arg = DeclareLaunchArgument(
        "use_foxglove_bridge",
        default_value="true",
        description="Launch Foxglove Bridge for the extensions and bundled layout",
    )
    use_cardinal_perception_sim_arg = DeclareLaunchArgument(
        "use_cardinal_perception_sim",
        default_value="true",
        description="Emit simulated /buoy_detections_3d once the boat is within ZED2i/Mid-360 "
                     "range+FOV of a cardinal marker, so cardinal_wall_publisher can build /virtual_obstacles",
    )
    use_local_ekf_arg = DeclareLaunchArgument("use_local_ekf", default_value="true")
    use_global_ekf_arg = DeclareLaunchArgument("use_global_ekf", default_value="true")
    task_type_arg = DeclareLaunchArgument(
        "task_type",
        default_value="task1",
        description="Waypoint set: 'task1' (competition scenario) or 'task1_follow' (lawnmower survey)",
    )
    start_at_wp3_arg = DeclareLaunchArgument(
        "start_at_wp3",
        default_value="false",
        description="Start Task1 sim 5 m before competition WP3 and navigate from WP3 onward",
    )
    sim_true_north_yaw_arg = DeclareLaunchArgument(
        "sim_true_north_yaw_rad",
        # True north is 30 deg east of map +Y (60 deg CCW from map +X).
        default_value="1.0471975511965976",
        description="Geographical true-north direction in Task1 sim map coordinates (rad)",
    )
    driver_delay_arg = DeclareLaunchArgument(
        "driver_delay",
        default_value="0.0",
        description="Delay before launching Task1 sensor/dynamics/EKF layer",
    )
    nav2_delay_arg = DeclareLaunchArgument(
        "nav2_delay",
        default_value="5.0",
        description="Delay before launching Nav2",
    )
    goal_delay_arg = DeclareLaunchArgument(
        "goal_delay",
        default_value="8.0",
        description="Delay before launching waypoint_publisher",
    )
    params_arg = DeclareLaunchArgument("params", default_value=config)
    nav2_params_arg = DeclareLaunchArgument("nav2_params", default_value=nav2_params)

    # ── SENSOR / PHYSICS LAYER (t=0) ─────────────────────────────────────────
    # SimNode is the sole simulation TF authority (publish_tf=True). Both EKFs
    # run with publish_tf=False so they only produce filtered odometry topics.
    dynamics = OpaqueFunction(function=lambda context: launch_dynamics(context, pkg_dutyed))

    sensor_noise_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sensor_noise, "launch", "sensor_noise.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("use_sensor_noise")),
        launch_arguments={
            # Match the GNSS topic consumed by the bundled Foxglove extension.
            "fix_topic": "/sensor/vehicle_gnss/fix/raw",
            "true_north_yaw_rad": LaunchConfiguration("sim_true_north_yaw_rad"),
        }.items(),
    )

    # Thruster driver: cmd_vel -> /thruster_command with P + DOB velocity control.
    # Replaces the deprecated open-loop `kinematics` node.
    thruster_driver_node = Node(
        package="thruster_driver",
        executable="thruster_driver_node",
        name="thruster_driver_node",
        parameters=[
            os.path.join(pkg_thruster, "config", "config.yaml"),
            {
                "robot_description": robot_description,
                "control.dob.enable": False,
                # The simulator consumes forces in [FR, FL, RR, RL] order,
                # while the shared driver is configured as [FL, FR, RL, RR].
                # Surge is invariant under that permutation, but sway and
                # yaw are reversed. Correct before allocation so a positive
                # Nav2 yaw command produces a positive simulated yaw rate;
                # otherwise the velocity feedback loop turns in place and
                # diverges continuously.
                "allocation.wrench_sign": [1.0, -1.0, -1.0],
                "thrusters.reverse": [False, False, False, False],
            },
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_thruster_driver")),
    )

    # Shared Nav2 publishes /cmd_vel_nav after smoothing and collision checking.
    # twist_mux is the sole publisher on /cmd_vel.
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        parameters=[
            os.path.join(pkg_robot, "config", "twist_mux.yaml"),
        ],
        remappings=[("cmd_vel_out", "/cmd_vel")],
        output="screen",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": False,
        }],
    )

    local_ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_local",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config", "task1_ekf_local.yaml"),
            {"publish_tf": False},
        ],
        remappings=[("odometry/filtered", "odometry/filtered/local")],
        condition=IfCondition(LaunchConfiguration("use_local_ekf")),
    )

    global_ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_global",
        output="screen",
        parameters=[
            os.path.join(pkg_robot, "config", "ekf_global.yaml"),
            {"publish_tf": False},
        ],
        remappings=[("odometry/filtered", "odometry/filtered/global")],
        condition=IfCondition(LaunchConfiguration("use_global_ekf")),
    )

    orchestrator = Node(
        package="task1_sim",
        executable="task1_orchestrator",
        name="task1_orchestrator",
        parameters=[LaunchConfiguration("params")],
        output="screen",
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
            "line_width": 0.08,
        }],
    )
    ground_speed = Node(
        package="tf_frame_arrow_publisher",
        executable="ground_speed_publisher",
        name="ground_speed_publisher",
        parameters=[{"odometry_topic": "/odom"}],
        output="screen",
    )
    gui_status_dummy_publisher = Node(
        package="task1_sim",
        executable="gui_status_dummy_publisher",
        name="gui_status_dummy_publisher",
        parameters=[{
            "battery_percent": 75.0,
            "cell_voltages": [4.00, 4.01, 4.00, 4.01],
            "temperature_c": 25.0,
            "control_status": "auto",
            "publish_rate_hz": 1.0,
        }],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_gui_dummy_publishers")),
    )
    foxglove_logger = Node(
        package="foxglove_logger",
        executable="foxglove_logger_node",
        name="foxglove_logger",
        parameters=[{
            "odometry_topic": "/odometry/filtered/global",
            "plan_topic": "/plan",
            "fix_topic": "/sensor/vehicle_gnss/fix/raw",
        }],
        output="screen",
    )
    foxglove_bridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("foxglove_bridge"),
                "launch",
                "foxglove_bridge_launch.xml",
            )
        ),
        condition=IfCondition(LaunchConfiguration("use_foxglove_bridge")),
    )

    validator = include_launch(
        "operation_validator",
        ["launch", "operation_validator.launch.py"],
        IfCondition(LaunchConfiguration("use_validator")),
    )

    cardinal_walls = OpaqueFunction(function=launch_cardinal_walls)
    cardinal_perception_sim = OpaqueFunction(function=launch_cardinal_perception_sim)

    sensor_layer_timer = TimerAction(
        period=LaunchConfiguration("driver_delay"),
        actions=[
            dynamics,
            sensor_noise_launch,
            thruster_driver_node,
            twist_mux,
            robot_state_pub_node,
            local_ekf_node,
            global_ekf_node,
            validator,
            orchestrator,
            cardinal_walls,
            cardinal_perception_sim,
        ],
    )

    # ── NAV2 LAYER (t=5s) ────────────────────────────────────────────────────
    nav2 = include_launch(
        "task1_sim",
        ["launch", "task1_navigation.launch.py"],
        IfCondition(LaunchConfiguration("use_nav2")),
        {
            "params_file": LaunchConfiguration("nav2_params"),
            "nav_through_poses_bt_xml": nav_through_poses_bt_xml,
            "nav_to_pose_bt_xml": nav_to_pose_bt_xml,
            "enable_diagnostics": "false",
        },
    )

    nav2_layer_timer = TimerAction(
        period=LaunchConfiguration("nav2_delay"),
        actions=[nav2],
    )

    # ── GOAL LAYER (t=8s) ────────────────────────────────────────────────────
    waypoints = include_launch(
        "waypoint_publisher",
        ["launch", "waypoint_publisher.launch.py"],
        IfCondition(LaunchConfiguration("use_waypoints")),
        {
            "task_type": LaunchConfiguration("task_type"),
            "frame_id": "map",
            "publish_rate_hz": "2.0",
            "waypoint_marker_topic": "/sim/task1_waypoint_markers",
            "nav2_goal_tolerance_m": "2.0",
            "start_competition_waypoint": PythonExpression([
                "'3' if '", LaunchConfiguration("start_at_wp3"), "' == 'true' else ''"
            ]),
            "show_waypoint_route_line": "false",
        },
    )

    goal_layer_timer = TimerAction(
        period=LaunchConfiguration("goal_delay"),
        actions=[waypoints],
    )

    startup_message = LogInfo(msg="========== Task1 Sim Bringup Started ==========")

    return LaunchDescription([
        use_dynamics_arg,
        use_nav2_arg,
        use_thruster_driver_arg,
        use_waypoints_arg,
        use_validator_arg,
        use_sensor_noise_arg,
        use_gui_dummy_publishers_arg,
        use_foxglove_bridge_arg,
        use_cardinal_perception_sim_arg,
        use_local_ekf_arg,
        use_global_ekf_arg,
        task_type_arg,
        start_at_wp3_arg,
        sim_true_north_yaw_arg,
        driver_delay_arg,
        nav2_delay_arg,
        goal_delay_arg,
        params_arg,
        nav2_params_arg,
        # Declare every configuration before an action can evaluate it.
        startup_message,
        heading_arrow,
        actual_route,
        ground_speed,
        gui_status_dummy_publisher,
        foxglove_logger,
        foxglove_bridge,
        sensor_layer_timer,
        nav2_layer_timer,
        goal_layer_timer,
    ])
