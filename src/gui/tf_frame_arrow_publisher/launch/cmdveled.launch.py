import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    pkg_robot = get_package_share_directory('tf_frame_arrow_publisher')

    tf_static_map2odom_node  = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_static_pub_lidar_to_base_link',
        output='screen',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'map', '--child-frame-id', 'odom']
    )

    arrow_visualization_node = Node(
        package='tf_frame_arrow_publisher',
        executable='cmdveled_tf_broadcaster',
        name='cmdveled_tf_broadcaster',
        output='screen',
    )

    return LaunchDescription([
        tf_static_map2odom_node,
        arrow_visualization_node,
    ])
