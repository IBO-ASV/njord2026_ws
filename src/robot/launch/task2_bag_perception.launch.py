"""Offline Task 2 perception with the Livox mounting TF supplied locally.

This launch deliberately excludes ``/tf_static`` from bag playback and
publishes the URDF's base_link -> livox_frame transform itself.  Do not use it
on the vessel: jetson_bringup already publishes that same fixed transform.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bag_path = LaunchConfiguration("bag_path")
    start_offset = LaunchConfiguration("start_offset")
    params = PathJoinSubstitution(
        [FindPackageShare("task2_perception"), "config", "task2_params.yaml"])
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("task2_perception"), "launch", "task2_perception.launch.py"])),
        launch_arguments={
            "use_sim_time": "true",
            "ego_odom_topic": "/odometry/filtered/local",
            "corridor_enabled": "false",
        }.items(),
    )
    pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("ship_perception_bringup"), "launch", "classical_pipeline.launch.py"])),
        launch_arguments={
            "lidar_topic": "/task2/points_filtered",
            "ego_odom_topic": "/odometry/filtered/local",
            "preprocessing_config_file": params,
            "segmentation_config_file": params,
            "tracker_config_file": params,
            "cluster_min_horizontal_size": LaunchConfiguration(
                "cluster_min_horizontal_size"),
            "cluster_min_radial_size": LaunchConfiguration(
                "cluster_min_radial_size"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("bag_path"),
        DeclareLaunchArgument("start_offset", default_value="0"),
        DeclareLaunchArgument("cluster_min_horizontal_size", default_value="0.0"),
        DeclareLaunchArgument("cluster_min_radial_size", default_value="-1.0"),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="bag_livox_static_tf_pub",
            arguments=[
                "--x", "0.5", "--y", "0.0", "--z", "0.8",
                "--roll", "3.141592653589793", "--pitch", "0.0", "--yaw", "0.0",
                "--frame-id", "base_link", "--child-frame-id", "livox_frame",
            ],
        ),
        perception,
        pipeline,
        ExecuteProcess(
            cmd=[
                "ros2", "bag", "play", bag_path, "--clock", "--start-offset", start_offset,
                "--topics", "/livox/lidar", "/odometry/filtered/local", "/tf",
                "/back_cam/camera_info", "/back_cam/image_raw",
                "/back_cam/image_raw/compressed",
            ],
            output="screen",
        ),
    ])
