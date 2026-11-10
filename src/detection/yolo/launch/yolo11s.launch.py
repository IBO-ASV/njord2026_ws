import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Same venv guard as the existing yolo.launch.py: ultralytics lives in a
    # dedicated Jetson venv, not in the system Python, so refuse to start
    # without an activated venv (see Docs/yolo11s_jetson_setup.md).
    if not os.environ.get('VIRTUAL_ENV'):
        raise RuntimeError(
            'YOLO11 launch requires the Jetson venv. Activate the YOLO venv '
            '(see Docs/yolo11s_jetson_setup.md) before running ros2 launch.'
        )

    pkg_name = 'yolo'
    pkg_share = get_package_share_directory(pkg_name)

    default_params_file = os.path.join(pkg_share, 'config', 'yolo11s_params.yaml')
    # The weights (yolo11s.pt / .engine) are NOT shipped in the repository;
    # a human must place them (see Docs/yolo11s_jetson_setup.md).
    default_model_path = os.path.join(pkg_share, 'config', 'yolo11s.pt')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to the yolo11s parameter YAML file'
    )

    yolo_model_path_arg = DeclareLaunchArgument(
        'yolo_model_path',
        default_value=default_model_path,
        description='Path to YOLO11s weights (.pt) or TensorRT engine '
                    '(.engine). The weight file is NOT shipped in the '
                    'repository.'
    )

    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cuda:0',
        description='Computation device (cpu or cuda:0). Ignored for .engine.'
    )

    backend_arg = DeclareLaunchArgument(
        'backend',
        default_value='pytorch',
        description='Expected model format: "pytorch" (.pt) or "tensorrt" '
                    '(.engine). The actual backend follows the model_path '
                    'extension.'
    )

    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='Input image topic'
    )

    yolo11_node = Node(
        package=pkg_name,
        executable='yolo11_node',
        name='yolo11_detector',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'model_path': LaunchConfiguration('yolo_model_path'),
                'device': LaunchConfiguration('device'),
                'backend': LaunchConfiguration('backend'),
                'camera_topic': LaunchConfiguration('camera_topic'),
            },
        ]
    )

    return LaunchDescription([
        params_file_arg,
        yolo_model_path_arg,
        device_arg,
        backend_arg,
        camera_topic_arg,
        yolo11_node,
    ])
