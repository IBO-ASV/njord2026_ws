import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_task2 = get_package_share_directory("task2_sim")
    pkg_robot = get_package_share_directory("robot")
    pkg_sensor_noise = get_package_share_directory("sensor_sim_with_noise")
    pkg_mppi = get_package_share_directory("asv_trajectory_planner")

    config = os.path.join(pkg_task2, "config", "task2_params.yaml")
    opponent_config = os.path.join(pkg_task2, "config", "task2_opponent_sim.yaml")
    robot_description_file = os.path.join(pkg_robot, "urdf", "robot.urdf_modified.urdf")
    robot_description = open(robot_description_file, "r").read()

    use_cmd_vel_kinematics = DeclareLaunchArgument("use_cmd_vel_kinematics", default_value="true")
    use_nav2 = DeclareLaunchArgument("use_nav2", default_value="true")
    use_mppi = DeclareLaunchArgument(
        "use_mppi", default_value="true",
        description="Run the recognition-assumed MPPI -> FollowPath chain",
    )
    params_arg = DeclareLaunchArgument("params", default_value=config)
    opponent_params_arg = DeclareLaunchArgument("opponent_params", default_value=opponent_config)
    driver_delay_arg = DeclareLaunchArgument(
        "driver_delay",
        default_value="0.0",
        description="Delay before launching simulation sensor/EKF layer",
    )
    nav2_delay_arg = DeclareLaunchArgument(
        "nav2_delay",
        default_value="5.0",
        description="Delay before launching Nav2",
    )
    goal_delay_arg = DeclareLaunchArgument(
        "goal_delay",
        default_value="8.0",
        description="Delay before launching the MPPI / FollowPath chain",
    )

    driver_delay = LaunchConfiguration("driver_delay")
    nav2_delay = LaunchConfiguration("nav2_delay")
    goal_delay = LaunchConfiguration("goal_delay")

    kinematic_plant = Node(
        package="task2_sim", executable="cmd_vel_kinematic_sim",
        name="cmd_vel_kinematic_sim", output="screen",
        condition=IfCondition(LaunchConfiguration("use_cmd_vel_kinematics")),
    )

    sensor_noise_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sensor_noise, "launch", "sensor_noise.launch.py")
        )
    )

    task2_orchestrator = Node(
        package="task2_sim",
        executable="task2_orchestrator",
        name="task2_orchestrator",
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
        }],
    )

    opponent_vessel = Node(
        package="task2_sim",
        executable="opponent_vessel_node",
        name="opponent_vessel_node",
        parameters=[LaunchConfiguration("opponent_params")],
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
            os.path.join(pkg_robot, "config", "ekf_local.yaml"),
            {"publish_tf": False},
        ],
        remappings=[("odometry/filtered", "odometry/filtered/local")],
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
    )

    navsat_transform_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform_node",
        output="screen",
        parameters=[{
            "frequency": 10.0,
            "magnetic_declination_radians": 0.0,
            "yaw_offset": 0.0,
            "zero_altitude": True,
            "broadcast_utm_transform": False,
            "publish_filtered_gps": True,
            "use_odometry_yaw": False,
            "wait_for_datum": False,
        }],
        remappings=[
            ("imu", "/wit/imu"),
            ("gps/fix", "/gps/fix"),
            ("odometry/filtered", "odometry/filtered/local"),
        ],
    )

    sensor_layer_timer = TimerAction(
        period=driver_delay,
        actions=[
            kinematic_plant,
            sensor_noise_launch,
            task2_orchestrator,
            opponent_vessel,
            robot_state_pub_node,
            local_ekf_node,
            global_ekf_node,
            navsat_transform_node,
        ],
    )

    # Same minimal FollowPath Nav2 stack as task2_autonomy.launch.py.  MPPI
    # sends /planned_path_pruned directly to ControllerServer's FollowPath
    # action; full Nav2 planning/BT servers are intentionally not launched.
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot, "launch", "navigation_launch_task2.py")
        ),
        launch_arguments={
            "params_file": os.path.join(pkg_robot, "config", "nav2_params_task2_jazzy.yaml"),
            "use_sim_time": "false",
            "autostart": "true",
            # No manual-control arbiter exists in simulation, so the
            # velocity-smoother output drives the simulated thruster driver.
            "auto_cmd_vel_topic": "/cmd_vel",
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_nav2")),
    )

    nav2_layer_timer = TimerAction(period=nav2_delay, actions=[nav2_launch])

    # Recognition-assumed bridge: the simulated opponent TF is converted to
    # the same /other_ship/twist interface that the real LiDAR tracker emits.
    mppi_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mppi, "launch", "planner_with_follow_path.launch.py")
        )
    )

    # Put the condition on the timer, rather than on the nested include.  A
    # condition on the include was not reliably expanded when it was executed
    # from TimerAction, leaving Nav2 active but no planner/follow-path nodes.
    goal_layer_timer = TimerAction(
        period=goal_delay,
        actions=[
            LogInfo(msg="Starting Task2 MPPI -> FollowPath chain"),
            mppi_launch,
        ],
        condition=IfCondition(LaunchConfiguration("use_mppi")),
    )

    return LaunchDescription([
        LogInfo(msg="========== Task2 Collision Avoidance Sim Bringup Started =========="),
        heading_arrow,
        actual_route,
        use_cmd_vel_kinematics,
        use_nav2,
        use_mppi,
        params_arg,
        opponent_params_arg,
        driver_delay_arg,
        nav2_delay_arg,
        goal_delay_arg,
        sensor_layer_timer,
        nav2_layer_timer,
        goal_layer_timer,
    ])
