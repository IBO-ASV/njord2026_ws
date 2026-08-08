"""Task 2 perception launch.

Starts the two task2_perception nodes with the unified task2_params.yaml.

NOTE: the pcl_segmentation submodule pipeline (classical_pipeline.launch.py
with lidar_topic:=/task2/points_filtered and
ego_odom_topic:=/odometry/filtered/local) is intentionally NOT included here;
the real-robot integration launch owns that composition.

Arguments (all default true):
  enable_cloud_filter       start task2_cloud_filter
  enable_opponent_selector  start opponent_selector

Frame, odometry, and simulated-clock arguments default to the real-vessel
configuration, but allow a rosbag integration launch to override them without
maintaining a second copy of either node definition.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare("task2_perception"), "config",
         "task2_params.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_cloud_filter", default_value="true",
            description="Start task2_cloud_filter (/livox/lidar -> "
                        "/task2/points_filtered)"),
        DeclareLaunchArgument(
            "enable_opponent_selector", default_value="true",
            description="Start opponent_selector (/tracked_objects -> "
                        "/other_ship/twist + opponent TF)"),
        DeclareLaunchArgument(
            "publish_self_marker", default_value="true",
            description="Publish the visualization-only self-vessel Marker"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "ego_odom_topic", default_value="/odometry/filtered/local"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument(
            "motion_filter_mode", default_value="standard",
            choices=["standard", "straight_line"],
            description="Opponent motion confidence gate mode."),

        Node(
            package="task2_perception",
            executable="task2_cloud_filter_node",
            name="task2_cloud_filter",
            output="screen",
            parameters=[
                params_file,
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "publish_self_marker": LaunchConfiguration(
                        "publish_self_marker"),
                },
            ],
            condition=IfCondition(LaunchConfiguration("enable_cloud_filter")),
        ),
        Node(
            package="task2_perception",
            executable="opponent_selector_node",
            name="opponent_selector",
            output="screen",
            parameters=[
                params_file,
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "ego_odom_topic": LaunchConfiguration("ego_odom_topic"),
                    "map_frame": LaunchConfiguration("map_frame"),
                    "base_frame": LaunchConfiguration("base_frame"),
                    "motion_filter_mode": LaunchConfiguration("motion_filter_mode"),
                },
            ],
            condition=IfCondition(
                LaunchConfiguration("enable_opponent_selector")),
        ),
    ])
