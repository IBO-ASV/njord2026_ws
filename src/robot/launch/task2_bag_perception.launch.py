"""Replay a Task 2 bag and run the complete LiDAR other-vessel pipeline.

This launch is deliberately perception-only: it never starts propulsion,
navigation, cameras, or physical sensors.  It starts a bag player, the three
TFs required by the recorded LiDAR, the Task 2 cloud filter, and the reusable
segmentation/tracking pipeline.

Example:
  ros2 launch robot task2_bag_perception.launch.py \\
    bag_path:=/path/to/rosbag_directory
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _include(package_name, launch_file, arguments):
    return GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare(package_name), "launch", launch_file])
                ),
                launch_arguments=arguments.items(),
            )
        ],
    )


def _bag_player(context):
    """Create rosbag's argument list after launch substitutions resolve."""
    storage_id = LaunchConfiguration("storage_id").perform(context)
    bag_path = LaunchConfiguration("bag_path").perform(context)
    playback_rate = LaunchConfiguration("playback_rate").perform(context)
    loop = LaunchConfiguration("loop").perform(context).lower() == "true"
    qos_profile = LaunchConfiguration("qos_profile").perform(context)

    # Leave storage selection to rosbag2 by default.  This supports both the
    # older MCAP test bags and SQLite bags recorded on Jazzy; forcing `mcap`
    # made an otherwise valid SQLite bag fail before playback began.
    command = [
        "ros2", "bag", "play",
    ]
    if storage_id:
        command.extend(["-s", storage_id])
    command.extend([
        bag_path,
        "--clock", "--rate", playback_rate,
        # ExecuteProcess has no interactive terminal.  rosbag otherwise tries
        # to install keyboard callbacks and exits with code 1 (tcgetattr
        # fails), leaving the perception chain without any LiDAR input.
        "--disable-keyboard-controls",
    ])
    if loop:
        command.append("--loop")
    if qos_profile:
        command.extend(["--qos-profile-overrides-path", qos_profile])
    return [ExecuteProcess(cmd=command, output="screen")]


def generate_launch_description():
    # This matches the temporary TF setup used for the July 2026 test bag.
    # Override any value when replaying a bag recorded with a different rig.
    static_tfs = [
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="bag_odom_to_base_footprint",
            arguments=["0", "0", "0", "0", "0", "0", "odom", "base_footprint"],
        ),
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="bag_base_footprint_to_base_link",
            arguments=["0", "0", "0", "0", "0", "0", "base_footprint", "base_link"],
        ),
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="bag_base_link_to_livox",
            arguments=["0.5", "0", "0.8", "0", "0", "3.141592653589793", "base_link", "livox_frame"],
        ),
    ]

    perception = _include(
        "task2_perception", "task2_perception.launch.py",
        {
            "enable_cloud_filter": "true",
            "enable_opponent_selector": "true",
            "use_sim_time": "true",
            "ego_odom_topic": "/odom",
            "map_frame": "odom",
            "base_frame": "base_link",
        },
    )
    tracking = _include(
        "ship_perception_bringup", "classical_pipeline.launch.py",
        {
            "lidar_topic": "/task2/points_filtered",
            "ego_odom_topic": "/odom",
            "preprocessing_config_file": PathJoinSubstitution(
                [FindPackageShare("task2_perception"), "config", "task2_params.yaml"]),
            "segmentation_config_file": PathJoinSubstitution(
                [FindPackageShare("task2_perception"), "config", "task2_params.yaml"]),
            "tracker_config_file": PathJoinSubstitution(
                [FindPackageShare("task2_perception"), "config", "task2_params.yaml"]),
        },
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "bag_path",
            default_value="/home/namikawa_arata/0_Njord/experiment/collision_avoidance_test_omu1",
            description="rosbag2 directory or MCAP file to replay",
        ),
        DeclareLaunchArgument(
            "storage_id", default_value="",
            description="Optional rosbag storage plugin. Empty = auto-detect "
                        "from metadata (recommended for MCAP and SQLite bags)."),
        DeclareLaunchArgument("playback_rate", default_value="1.0"),
        DeclareLaunchArgument(
            "loop", default_value="true",
            description="Non-empty value repeats the bag after its end",
        ),
        DeclareLaunchArgument(
            "qos_profile",
            default_value=PathJoinSubstitution(
                [FindPackageShare("robot"), "config", "collision_avoidance_qos.yaml"]
            ),
            description="rosbag QoS override YAML path (set to an empty string to disable)",
        ),
        *static_tfs,
        perception,
        tracking,
        OpaqueFunction(function=_bag_player),
    ])
