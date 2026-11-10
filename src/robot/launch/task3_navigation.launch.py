import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_robot = get_package_share_directory("robot")
    pkg_waypoint = get_package_share_directory("waypoint_publisher")

    task_type_arg = DeclareLaunchArgument(
        "task_type",
        default_value="task3_1",
        description="Task3 mode: task3_1 or task3_2.",
    )
    enable_dynamic_gate_arg = DeclareLaunchArgument(
        "use_dynamic_gate_midpoints",
        default_value="true",
        description="Use live red/green buoy TF midpoints for Task3 gate waypoints.",
    )
    full_sequence_arg = DeclareLaunchArgument(
        "run_full_sequence",
        default_value="false",
        description="For task3_1, continue through task3_2 and finish at GPS10.",
    )

    field_boundary_node = Node(
        package="buoy_obstacle_publisher",
        executable="field_boundary_publisher",
        name="field_boundary_publisher",
        parameters=[{
            "map_frame": "odom",
            "resolution": 0.2,
            "map_size_m": 80.0,
            # Operating bounds: x=[-24, 24], y=[-14, 14].
            "field_size_x_m": 48.0,
            "field_size_y_m": 28.0,
            "field_center_x": 0.0,
            "field_center_y": 0.0,
            "boundary_cost": 100,
            "include_task3_docks": True,
            "dock_wall_thickness_m": 0.3,
        }],
        output="screen",
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot, "launch", "nav2.launch.py")
        ),
        launch_arguments={
            "params_file": os.path.join(pkg_robot, "config", "nav2_params_task3_humble.yaml"),
        }.items(),
    )

    waypoint_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_waypoint, "launch", "waypoint_publisher.launch.py")
        ),
        launch_arguments={
            "task_type": LaunchConfiguration("task_type"),
            "frame_id": "odom",
            "publish_rate_hz": "2.0",
            "use_dynamic_gate_midpoints": LaunchConfiguration("use_dynamic_gate_midpoints"),
            "run_full_sequence": LaunchConfiguration("run_full_sequence"),
        }.items(),
    )

    return LaunchDescription([
        task_type_arg,
        enable_dynamic_gate_arg,
        full_sequence_arg,
        field_boundary_node,
        nav2_launch,
        waypoint_publisher,
    ])
