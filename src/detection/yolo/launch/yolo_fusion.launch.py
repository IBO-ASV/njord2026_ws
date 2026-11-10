import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the preserved camera-LiDAR fusion implementation.

    The normal yolo.launch.py remains the remote/test07089 implementation.
    This launch file is intentionally separate so both implementations can
    be tested without replacing the production entry point.
    """
    pkg_share = get_package_share_directory('yolo')
    default_config_path = os.path.join(pkg_share, 'config', 'yolo_params.yaml')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_path,
        description='Path to the camera-LiDAR fusion YOLO parameter YAML',
    )

    fusion_node = Node(
        package='yolo',
        executable='yolo_fusion_node',
        name='yolo_fusion_detector',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
    )

    return LaunchDescription([
        config_file_arg,
        fusion_node,
    ])
