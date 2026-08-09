"""Resident Task 2 support graph for the persistent Mission/Control stack.

This is intentionally an adapter, not the historical ``task2_autonomy``
bringup.  It starts no hardware, Nav2, command arbiter, or thruster process.
Those are owned by the role bringup and retain the only canonical command path:

  Task 2 MPPI -> FollowPath -> Nav2 controller -> collision monitor
  -> /cmd_vel_nav -> Command Arbiter -> /cmd_vel

``follow_path_client_node`` remains disabled until Mission Manager publishes a
true value on ``/mission/task2/enabled``.  Cancelling the task publishes false
and cancels its outstanding FollowPath goal.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include(package_name, launch_file, arguments=None):
    return GroupAction(
        scoped=True,
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
    own_odom_topic = LaunchConfiguration("own_odom_topic")
    enable_ship_tracking = LaunchConfiguration("enable_ship_tracking")
    motion_mode = LaunchConfiguration("opponent_motion_mode")

    perception = _include(
        "task2_perception",
        "task2_perception.launch.py",
        {
            "enable_cloud_filter": "true",
            "enable_opponent_selector": enable_ship_tracking,
            "publish_self_marker": "false",
            "ego_odom_topic": own_odom_topic,
            "motion_filter_mode": motion_mode,
        },
    )
    tracking = _include(
        "ship_perception_bringup",
        "classical_pipeline.launch.py",
        {
            "lidar_topic": "/task2/points_filtered",
            "ego_odom_topic": own_odom_topic,
            "motion_mode": motion_mode,
            "preprocessing_config_file": PathJoinSubstitution(
                [FindPackageShare("task2_perception"), "config", "task2_params.yaml"]
            ),
            "segmentation_config_file": PathJoinSubstitution(
                [FindPackageShare("task2_perception"), "config", "task2_params.yaml"]
            ),
            "tracker_config_file": PathJoinSubstitution(
                [FindPackageShare("task2_perception"), "config", "task2_params.yaml"]
            ),
        },
    )
    mppi = _include(
        "asv_trajectory_planner",
        "planner_real.launch.py",
        {"own_odom_topic": own_odom_topic, "mission_gate_required": "true"},
    )
    return LaunchDescription([
        DeclareLaunchArgument("own_odom_topic", default_value="/odometry/filtered/global"),
        DeclareLaunchArgument("enable_ship_tracking", default_value="true"),
        DeclareLaunchArgument(
            "opponent_motion_mode", default_value="straight_line",
            choices=["standard", "straight_line"],
        ),
        perception,
        GroupAction(condition=IfCondition(enable_ship_tracking), actions=[tracking]),
        mppi,
    ])
