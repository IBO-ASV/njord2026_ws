import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_robot     = get_package_share_directory("robot")
    pkg_wit       = get_package_share_directory("wit_node")
    pkg_um982     = get_package_share_directory("um982_driver")
    pkg_waypoint  = get_package_share_directory("waypoint_publisher")
    pkg_thruster  = get_package_share_directory("thruster_driver")
    robot_description_file = os.path.join(pkg_robot, "urdf", "robot.urdf_modified.urdf")
    robot_description = open(robot_description_file, "r").read()

    # Task1-2: cardinal-marker recognition / route generation experiment
    # (issue #46). Route waypoints for this sub-task are still driven live by
    # the marker-recognition perception pipeline, not by waypoint_publisher's
    # static task1_waypoints.yaml (waypoint_publisher_node's TaskType enum has
    # no dedicated task1_2 entry yet), so the GOAL layer defaults to disabled.
    # Pass -r use_waypoints:=true to fall back to the shared task1 GPS route
    # once/if a task1_2 waypoint set lands.
    driver_delay_arg = DeclareLaunchArgument(
        "driver_delay", default_value="0.0",
        description="Delay before launching the sensor/localization layer")
    nav2_delay_arg = DeclareLaunchArgument(
        "nav2_delay", default_value="5.0",
        description="Delay before launching Nav2 (needs GLIM TF and filtered odometry)")
    goal_delay_arg = DeclareLaunchArgument(
        "goal_delay", default_value="8.0",
        description="Delay before launching waypoint_publisher (needs Nav2 action servers up)")
    use_waypoints_arg = DeclareLaunchArgument(
        "use_waypoints", default_value="false",
        description="Fall back to the shared task1 static waypoint route instead of "
                     "marker-driven navigation")
    enable_diagnostics_arg = DeclareLaunchArgument(
        "enable_diagnostics", default_value="true",
        description="Launch generic topic heartbeat diagnostics")
    legacy_graph_arg = DeclareLaunchArgument(
        "enable_legacy_graph", default_value="false",
        description=(
            "Compatibility only: start this historical hardware/Nav2 graph. "
            "Keep false when persistent role bringup is running."))

    driver_delay = LaunchConfiguration("driver_delay")
    nav2_delay = LaunchConfiguration("nav2_delay")
    goal_delay = LaunchConfiguration("goal_delay")
    enable_diagnostics = LaunchConfiguration("enable_diagnostics")

    # ── SENSOR / LOCALIZATION LAYER (t=0) ────────────────────────────────────
    wit_imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_wit, "launch", "wit.launch.py")))

    um982_gnss = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_um982, "launch", "um982.launch.py")))

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot, "launch", "localization.launch.py")),
        launch_arguments={"enable_diagnostics": enable_diagnostics}.items())

    thruster_driver_node = Node(
        package="thruster_driver",
        executable="thruster_driver_node",
        name="thruster_driver_node",
        parameters=[
            os.path.join(pkg_thruster, "config", "config.yaml"),
            {
                "robot_description": robot_description,
                "transport_mode": "can",
            },
        ],
        output="screen",
    )

    sensor_layer_timer = TimerAction(
        period=driver_delay,
        actions=[
            wit_imu,
            um982_gnss,
            localization,
            thruster_driver_node,
        ]
    )

    # ── NAV2 LAYER ────────────────────────────────────────────────────────────
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot, "launch", "nav2.launch.py")),
        launch_arguments={
            "params_file": os.path.join(pkg_robot, "config", "nav2_params_humble.yaml"),
            "enable_diagnostics": enable_diagnostics,
        }.items()
    )

    nav2_layer_timer = TimerAction(period=nav2_delay, actions=[nav2_launch])

    # ── GOAL LAYER (optional static fallback route) ─────────────────────────
    waypoint_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_waypoint, "launch", "waypoint_publisher.launch.py")),
        launch_arguments={
            "task_type": "task1",
            "frame_id": "map",
            "publish_rate_hz": "2.0",
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_waypoints")),
    )

    goal_layer_timer = TimerAction(period=goal_delay, actions=[waypoint_publisher])

    startup_message = LogInfo(
        msg="========== Task1-2 (Cardinal Marker Recognition) Bringup Started ==========")

    return LaunchDescription([
        driver_delay_arg,
        nav2_delay_arg,
        goal_delay_arg,
        use_waypoints_arg,
        enable_diagnostics_arg,
        legacy_graph_arg,
        GroupAction(
            condition=IfCondition(LaunchConfiguration("enable_legacy_graph")),
            actions=[startup_message, sensor_layer_timer, nav2_layer_timer, goal_layer_timer],
        ),
    ])
