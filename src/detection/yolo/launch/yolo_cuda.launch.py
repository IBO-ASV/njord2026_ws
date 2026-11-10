import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'yolo'
    pkg_share = get_package_share_directory(pkg_name)

    default_model_path = os.path.join(pkg_share, 'config', 'best.pt')

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=default_model_path,
        description='Absolute path to the YOLO model file (.pt)'
    )

    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cuda:0',
        description='Computation device (default: cuda:0)'
    )

    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='Image topic consumed by the YOLO detector',
    )

    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='yolo_detector_cuda',
        description='Name assigned to the YOLO detector node',
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for YOLO outputs such as yolo/debug_image',
    )
    enable_roi_arg = DeclareLaunchArgument(
        'enable_roi', default_value='true',
        description='Publish buoy ROI messages from detections',
    )
    enable_virtual_wall_arg = DeclareLaunchArgument(
        'enable_virtual_wall', default_value='false',
        description='Publish virtual obstacles from detections',
    )
    enable_debug_image_arg = DeclareLaunchArgument(
        'enable_debug_image', default_value='true',
        description='Publish annotated debug images when subscribed',
    )

    yolo_node = Node(
        package=pkg_name,
        executable='yolo_cuda_node',
        name=LaunchConfiguration('node_name'),
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'device': LaunchConfiguration('device'),
            'camera_topic': LaunchConfiguration('camera_topic'),
            'enable_roi': LaunchConfiguration('enable_roi'),
            'enable_virtual_wall': LaunchConfiguration('enable_virtual_wall'),
            'enable_debug_image': LaunchConfiguration('enable_debug_image'),
        }]
    )

    return LaunchDescription([
        model_path_arg,
        device_arg,
        camera_topic_arg,
        node_name_arg,
        namespace_arg,
        enable_roi_arg,
        enable_virtual_wall_arg,
        enable_debug_image_arg,
        yolo_node,
    ])
