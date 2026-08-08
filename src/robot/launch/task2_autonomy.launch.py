"""Task 2 autonomy overlay for a running manual_control.launch.py.

This launch deliberately owns no hardware or final actuator nodes.  Start
``simple_manual/manual_control.launch.py`` first; it owns sensor drivers,
manual joystick control, command_arbiter, thruster_driver and serial_writer.

This overlay consumes the topics already provided by that launch and adds:

  /livox/lidar + ego odometry -> Task 2 ship perception -> /other_ship/twist
  ego odometry + opponent + task waypoints -> MPPI -> /planned_path_pruned
  /planned_path_pruned -> Nav2 FollowPath -> /cmd_vel_nav

The command_arbiter from manual_control receives /cmd_vel_nav through its
existing default, and publishes the selected
manual or automatic command on /cmd_vel.

Prerequisites from the existing manual-control stack are /livox/lidar, ego
odometry (default: /odom), and the map/odom/base_link TF chain.  This launch
does not create substitute odometry or TF data.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _include(package_name, launch_file, *, condition=None, arguments=None):
    return GroupAction(
        scoped=True,
        condition=condition,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare(package_name), "launch", launch_file])
                ),
                launch_arguments=(arguments or {}).items(),
            )
        ],
    )


def generate_launch_description():
    enable_lidar = LaunchConfiguration("enable_lidar")
    enable_ship_tracking = LaunchConfiguration("enable_ship_tracking")
    enable_nav2 = LaunchConfiguration("enable_nav2")
    own_odom_topic = LaunchConfiguration("own_odom_topic")
    opponent_motion_mode = LaunchConfiguration("opponent_motion_mode")

    task2_perception = _include(
        "task2_perception",
        "task2_perception.launch.py",
        condition=IfCondition(enable_lidar),
        arguments={
            "enable_cloud_filter": "true",
            "enable_opponent_selector": enable_ship_tracking,
            "publish_self_marker": "false",
            "motion_filter_mode": opponent_motion_mode,
        },
    )
    ship_tracking = _include(
        "ship_perception_bringup",
        "classical_pipeline.launch.py",
        condition=IfCondition(enable_ship_tracking),
        arguments={
            "lidar_topic": "/task2/points_filtered",
            "ego_odom_topic": own_odom_topic,
            "motion_mode": opponent_motion_mode,
            "preprocessing_config_file": PathJoinSubstitution(
                [FindPackageShare("task2_perception"), "config", "task2_params.yaml"]),
            "segmentation_config_file": PathJoinSubstitution(
                [FindPackageShare("task2_perception"), "config", "task2_params.yaml"]),
            "tracker_config_file": PathJoinSubstitution(
                [FindPackageShare("task2_perception"), "config", "task2_params.yaml"]),
        },
    )
    mppi = _include(
        "asv_trajectory_planner",
        "planner_real.launch.py",
        arguments={"own_odom_topic": own_odom_topic},
    )
    nav2 = _include(
        "robot",
        "navigation_launch_task2.py",
        condition=IfCondition(enable_nav2),
        arguments={
            "params_file": PathJoinSubstitution(
                [FindPackageShare("robot"), "config", "nav2_params_task2_jazzy.yaml"]
            ),
            "use_sim_time": "false",
            "autostart": "true",
            "auto_cmd_vel_topic": "/cmd_vel_nav",
        },
    )
    autonomy_ready = Node(
        package="asv_trajectory_planner",
        executable="task2_autonomy_ready_node",
        name="task2_autonomy_ready",
        output="screen",
        parameters=[{
            "path_topic": "/planned_path_pruned",
            "action_name": "/follow_path",
            "path_timeout_sec": 2.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("enable_lidar", default_value="true"),
        DeclareLaunchArgument("enable_ship_tracking", default_value="true"),
        DeclareLaunchArgument("enable_nav2", default_value="true"),
        DeclareLaunchArgument(
            "opponent_motion_mode", default_value="straight_line",
            choices=["standard", "straight_line"],
            description="standard: normal vessel tracking; straight_line: "
                        "Task 2 constant-velocity confidence gates."),
        DeclareLaunchArgument(
            "own_odom_topic", default_value="/odom",
            description="Ego odometry already published by the manual-control stack.",
        ),
        task2_perception,
        ship_tracking,
        mppi,
        nav2,
        autonomy_ready,
    ])
