#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Launch file for waypoint_publisher node
Supports task1, task2, task2_bak, task3_1, and task3_2 modes
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description for waypoint_publisher"""
    
    # Declare arguments
    task_type_arg = DeclareLaunchArgument(
        'task_type',
        default_value='task1',
        description=(
            'Task type: task1, task1_skip_1_1, task1_follow, task2, '
            'task2_bak, task3_1, or task3_2'
        )
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='Frame ID for waypoints'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate_hz',
        default_value='2.0',
        description='Publish rate in Hz'
    )

    dynamic_gate_arg = DeclareLaunchArgument(
        'use_dynamic_gate_midpoints',
        default_value='true',
        description='Use live red/green buoy TF midpoints for Task3 gate waypoints'
    )

    full_sequence_arg = DeclareLaunchArgument(
        'run_full_sequence',
        default_value='false',
        description='For task3_1, continue through task3_2 and finish at GPS10'
    )

    waypoint_marker_topic_arg = DeclareLaunchArgument(
        'waypoint_marker_topic', default_value='/waypoint_markers',
        description='MarkerArray topic used to visualize the configured waypoint sequence')
    nav2_goal_tolerance_arg = DeclareLaunchArgument(
        'nav2_goal_tolerance_m', default_value='1.0',
        description='Nav2 XY reach tolerance rendered around each waypoint in metres')
    waypoint_route_line_arg = DeclareLaunchArgument(
        'show_waypoint_route_line', default_value='false',
        description='Draw a line connecting waypoint markers')
    start_competition_waypoint_arg = DeclareLaunchArgument(
        'start_competition_waypoint', default_value='',
        description='Competition waypoint label from which to start the route (Task1 sim)')
    
    # Create node
    waypoint_publisher_node = Node(
        package='waypoint_publisher',
        executable='waypoint_publisher_node',
        name='waypoint_publisher',
        parameters=[
            {
                'task_type': LaunchConfiguration('task_type'),
                'frame_id': LaunchConfiguration('frame_id'),
                'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
                'use_dynamic_gate_midpoints': ParameterValue(
                    LaunchConfiguration('use_dynamic_gate_midpoints'),
                    value_type=bool,
                ),
                'run_full_sequence': ParameterValue(
                    LaunchConfiguration('run_full_sequence'),
                    value_type=bool,
                ),
                'waypoint_marker_topic': LaunchConfiguration('waypoint_marker_topic'),
                'nav2_goal_tolerance_m': ParameterValue(
                    LaunchConfiguration('nav2_goal_tolerance_m'), value_type=float),
                'show_waypoint_route_line': ParameterValue(
                    LaunchConfiguration('show_waypoint_route_line'), value_type=bool),
                # A numeric label such as "3" is otherwise inferred as an
                # integer by the ROS parameter parser.  The node uses the
                # competition label as a string to select the route slice.
                'start_competition_waypoint': ParameterValue(
                    LaunchConfiguration('start_competition_waypoint'), value_type=str),
            }
        ],
        output='screen',
        emulate_tty=True,
    )
    
    # Create launch description
    ld = LaunchDescription([
        task_type_arg,
        frame_id_arg,
        publish_rate_arg,
        dynamic_gate_arg,
        full_sequence_arg,
        waypoint_marker_topic_arg,
        nav2_goal_tolerance_arg,
        waypoint_route_line_arg,
        start_competition_waypoint_arg,
        waypoint_publisher_node,
    ])
    
    return ld
