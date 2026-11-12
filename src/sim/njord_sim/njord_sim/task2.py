"""Task2 CPU MPPI + the production point-cloud tracking pipeline."""

from pathlib import Path
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory as share


def nodes(directory, seed):
    params = str(Path(share("task2_perception")) / "config/task2_params.yaml")
    classical = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(share("ship_perception_bringup"))
                / "launch/classical_pipeline.launch.py"
            )
        ),
        launch_arguments={
            "lidar_topic": "/task2/points_filtered",
            "ego_odom_topic": "/odometry/filtered/local",
            "preprocessing_config_file": params,
            "segmentation_config_file": params,
            "tracker_config_file": params,
        }.items(),
    )
    return [
        classical,
        Node(
            package="task2_perception",
            executable="task2_cloud_filter_node",
            name="task2_cloud_filter",
            parameters=[params],
            output="screen",
        ),
        Node(
            package="task2_perception",
            executable="opponent_selector_node",
            name="opponent_selector",
            parameters=[
                params,
                {
                    "map_frame": "odom",
                    "min_absolute_speed_knots": 0.5,
                    "corridor_enabled": True,
                    "corridor_start_topic": "/waypoint1_pose",
                    "corridor_end_topic": "/waypoint2_pose",
                    "corridor_ignore_left_side": False,
                    "clip_opponent_bearing_to_corridor": False,
                },
            ],
            output="screen",
        ),
        Node(
            package="asv_trajectory_planner",
            executable="task2_waypoint_pose_publisher",
            parameters=[{"frame_id": "odom"}],
            output="screen",
        ),
        Node(
            package="asv_trajectory_planner",
            executable="planner_node",
            name="planner_node",
            parameters=[
                {
                    "own_odom_topic": "/odometry/filtered/local",
                    "other_ship_twist_topic": "/other_ship/twist",
                    "other_twist_is_relative": False,
                    "other_heading_from_velocity": True,
                    "path_guard_enabled": True,
                    "path_topic": "/planned_path_pruned",
                    "frame_id": "odom",
                    "other_ship_frame": "opponent_vessel",
                    "own_frame": "base_link",
                    "planning_frequency": 2.0,
                    "require_other_ship": False,
                    "require_opponent_detection_status": True,
                    "use_detected_buoys": True,
                    "mppi.buoy_cost_weight": 120.0,
                    "mppi.num_samples": 512,
                    "mppi.horizon": 150,
                    "mppi.dt": 0.15,
                    "mppi.target_speed": 1.0288888889,
                    "mppi.path_cost_weight": 150.0,
                    "mppi.starboard_bias_weight": 300.0,
                    "mppi.collision_cost_min": 10.0,
                    "mppi.collision_cost_max": 50.0,
                    "mppi.own_loa_m": 1.5,
                    "avoid_radius": 4.0,
                    "avoid_offset": 5.0,
                    "opponent_use_distance_m": 40.0,
                    "mppi.random_seed": seed,
                    "point_spacing": 0.5,
                }
            ],
            output="screen",
            additional_env={"OMP_NUM_THREADS": "2", "OPENBLAS_NUM_THREADS": "2"},
        ),
        Node(
            package="asv_trajectory_planner",
            executable="follow_path_client_node",
            name="task2_follow_path_client",
            parameters=[
                {
                    "path_topic": "/planned_path_pruned",
                    "path_permission_topic": "/mission/task2/path_safe",
                    "mission_gate_required": True,
                    "enabled_topic": "/mission/task2/enabled",
                    "goal_reached_topic": "/mission/task2/goal_reached",
                    "enable_replanning": True,
                    "send_frequency": 1.0,
                }
            ],
            output="screen",
        ),
    ]
