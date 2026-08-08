from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    task2_gps_waypoint_publisher = Node(
        package="asv_trajectory_planner",
        executable="task2_gps_waypoint_publisher",
        name="task2_gps_waypoint_publisher",
        output="screen",
        parameters=[
            {
                "gps_marker_topic": "/sim/task2_gps_markers",
                "waypoint1_topic": "/waypoint1_pose",
                "waypoint2_topic": "/waypoint2_pose",
                "frame_id": "map",
                "publish_frequency": 2.0,
            }
        ],
    )

    opponent_twist_from_tf_node = Node(
        package="asv_trajectory_planner",
        executable="opponent_twist_from_tf_node",
        name="opponent_twist_from_tf_node",
        output="screen",
        parameters=[
            {
                "parent_frame": "map",
                "child_frame": "opponent_vessel",
                "twist_topic": "/other_ship/twist",
                "publish_rate_hz": 10.0,
                "max_dt_s": 1.0,
            }
        ],
    )

    planner_node = Node(
        package="asv_trajectory_planner",
        executable="planner_node",
        name="planner_node",
        output="screen",
        parameters=[
            {
                "own_odom_topic": "/odom",
                "other_ship_twist_topic": "/other_ship/twist",
                "waypoint1_topic": "/waypoint1_pose",
                "waypoint2_topic": "/waypoint2_pose",
                "path_topic": "/planned_path_pruned",

                "own_frame": "base_link",
                "other_ship_frame": "opponent_vessel",

                "frame_id": "map",
                "planning_frequency": 2.0,
                "point_spacing": 0.5,
                "avoid_radius": 2.0,
                "avoid_offset": 3.0,

                # 相手船がいれば見る。いなければ相手船なしでPathを出す。
                "require_other_ship": False,

                # /other_ship/twist は map基準で出す
                "other_twist_is_relative": False,
                "opponent_use_distance_m": 20.0,
                "opponent_passed_margin_m": 10.0,
                "reconnect_line_distance_m": 1.0,
                "reconnect_ahead_length_m": 5.0,
                "straight_path_spacing_m": 2.0,
                "straight_path_length_m": 60.0,
                "mppi_smoothing_window": 3,
            }
        ],
    )
    path_pruner_node = Node(
        package="asv_trajectory_planner",
        executable="path_pruner_node",
        name="path_pruner_node",
        output="screen",
        parameters=[{
            "input_path_topic": "/planned_path",
            "output_path_topic": "/planned_path_pruned",
            "skip_points_after_closest": 0,
            "prepend_current_pose": True,
            "min_output_points": 3,
            "min_point_spacing_m": 0.5,
            "odom_topic": "/odom",
                                            }],
    )



    follow_path_client_node = Node(
        package="asv_trajectory_planner",
        executable="follow_path_client_node",
        name="follow_path_client_node",
        output="screen",
        parameters=[
            {
                "path_topic": "/planned_path_pruned",
                "action_name": "/follow_path",
                "controller_id": "FollowPath",
                "goal_checker_id": "general_goal_checker",
                "progress_checker_id": "progress_checker",

                # goalを送り直しすぎるとNav2がabortしやすいので抑制
                "send_frequency": 1.0,
                "enable_replanning": True,
            }
        ],
    )
    return LaunchDescription(
        [
            task2_gps_waypoint_publisher,
            opponent_twist_from_tf_node,
            planner_node,
            path_pruner_node,
            follow_path_client_node,
        ]
    )
