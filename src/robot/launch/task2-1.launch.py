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

    # Task2-1: collision avoidance experiment (issue #47).
    # task2_sim.launch.py's opponent_vessel_node/ideal_lidar_pointcloud_node
    # only exist to fabricate an obstacle for the simulator; on real hardware
    # obstacle detection comes from the live perception stack (e.g.
    # src/detection/pcl_segmentation) running independently, so it is not
    # part of this bringup.
    driver_delay_arg = DeclareLaunchArgument(
        "driver_delay", default_value="0.0",
        description="Delay before launching the sensor/localization layer")
    nav2_delay_arg = DeclareLaunchArgument(
        "nav2_delay", default_value="5.0",
        description="Delay before launching Nav2 (needs GLIM TF and filtered odometry)")
    goal_delay_arg = DeclareLaunchArgument(
        "goal_delay", default_value="8.0",
        description="Delay before launching waypoint_publisher (needs Nav2 action servers up)")
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
            "params_file": os.path.join(pkg_robot, "config", "nav2_params_task2_humble.yaml"),
            "enable_diagnostics": enable_diagnostics,
        }.items()
    )

    nav2_layer_timer = TimerAction(period=nav2_delay, actions=[nav2_launch])

    # ── GOAL LAYER ────────────────────────────────────────────────────────────
    waypoint_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_waypoint, "launch", "waypoint_publisher.launch.py")),
        launch_arguments={
            "task_type": "task2",
            "frame_id": "map",
            "publish_rate_hz": "2.0",
        }.items()
    )

    goal_layer_timer = TimerAction(period=goal_delay, actions=[waypoint_publisher])

    startup_message = LogInfo(
        msg="========== Task2-1 (Collision Avoidance) Bringup Started ==========")

    return LaunchDescription([
        driver_delay_arg,
        nav2_delay_arg,
        goal_delay_arg,
        enable_diagnostics_arg,
        legacy_graph_arg,
        GroupAction(
            condition=IfCondition(LaunchConfiguration("enable_legacy_graph")),
            actions=[startup_message, sensor_layer_timer, nav2_layer_timer, goal_layer_timer],
        ),
    ])
