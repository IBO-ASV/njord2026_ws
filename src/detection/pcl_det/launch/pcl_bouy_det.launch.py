#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Launch the PCL buoy detection node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description for pcl_bouy_det."""
    # Declare arguments
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/livox/lidar',
        description='Input PointCloud2 topic'
    )

    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/buoy_detections',
        description='Output Point2D (PointStamped) topic'
    )

    roi_topic_arg = DeclareLaunchArgument(
        'roi_topic',
        default_value='/buoy_roi',
        description='Input ROI topic from YOLO'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='base_link',
        description='Frame ID for TF transformations'
    )

    height_min_arg = DeclareLaunchArgument(
        'height_min',
        default_value='-1.0',
        description='Minimum height filter (m)'
    )

    height_max_arg = DeclareLaunchArgument(
        'height_max',
        default_value='0.5',
        description='Maximum height filter (m)'
    )

    distance_threshold_arg = DeclareLaunchArgument(
        'distance_threshold',
        default_value='0.05',
        description='Clustering distance threshold (m)'
    )

    min_points_arg = DeclareLaunchArgument(
        'min_points_per_cluster',
        default_value='5',
        description='Minimum points per cluster'
    )

    roi_timeout_arg = DeclareLaunchArgument(
        'roi_timeout_sec',
        default_value='0.5',
        description='ROI timeout in seconds'
    )

    # Create node
    pcl_bouy_det_node = ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container_mt',
        name='pcl_bouy_det_container',
        namespace='/',
        output='screen',
        emulate_tty=True,
        composable_node_descriptions=[
            ComposableNode(
                package='pcl_det',
                plugin='pcl_det::PclBuoyDetectionNode',
                name='pcl_bouy_det_node',
                parameters=[
                    {
                        'input_topic': LaunchConfiguration('input_topic'),
                        'roi_topic': LaunchConfiguration('roi_topic'),
                        'output_topic': LaunchConfiguration('output_topic'),
                        'frame_id': LaunchConfiguration('frame_id'),
                        'height_min': LaunchConfiguration('height_min'),
                        'height_max': LaunchConfiguration('height_max'),
                        'distance_threshold': LaunchConfiguration('distance_threshold'),
                        'min_points_per_cluster': LaunchConfiguration('min_points_per_cluster'),
                        'roi_timeout_sec': LaunchConfiguration('roi_timeout_sec'),
                    }
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        ],
    )

    return LaunchDescription([
        input_topic_arg,
        output_topic_arg,
        roi_topic_arg,
        frame_id_arg,
        height_min_arg,
        height_max_arg,
        distance_threshold_arg,
        min_points_arg,
        roi_timeout_arg,
        pcl_bouy_det_node,
    ])
