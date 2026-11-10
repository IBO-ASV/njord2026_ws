import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    if not os.environ.get('VIRTUAL_ENV'):
        raise RuntimeError(
            'YOLO launch requires the Jetson venv. Activate .venv and source export_python_path.sh '
            'before running ros2 launch.'
        )

    pkg_name = 'yolo'
    pkg_share = get_package_share_directory(pkg_name)

    # デフォルトのモデルパス
    default_model_path = os.path.join(pkg_share, 'config', 'best.pt')

    # Launch引数の宣言
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=default_model_path,
        description='Absolute path to the YOLO model file (.pt)'
    )
    
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cpu',
        description='Computation device (cpu or cuda:0)'
    )

    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='Image topic consumed by the YOLO detector',
    )
    publish_detections_arg = DeclareLaunchArgument(
        'publish_detections', default_value='false',
        description='Publish vision_msgs/Detection2DArray for camera-LiDAR fusion',
    )
    detections_topic_arg = DeclareLaunchArgument(
        'detections_topic', default_value='/yolo/detections',
        description='Output topic for camera-LiDAR fusion detections',
    )
    use_image_roi_arg = DeclareLaunchArgument(
        'use_image_roi', default_value='false',
        description='Run inference only in the normalized image ROI parameters below',
    )
    roi_x_min_ratio_arg = DeclareLaunchArgument('roi_x_min_ratio', default_value='0.0')
    roi_x_max_ratio_arg = DeclareLaunchArgument('roi_x_max_ratio', default_value='1.0')
    roi_y_min_ratio_arg = DeclareLaunchArgument('roi_y_min_ratio', default_value='0.0')
    roi_y_max_ratio_arg = DeclareLaunchArgument('roi_y_max_ratio', default_value='1.0')
    enable_color_estimation_arg = DeclareLaunchArgument(
        'enable_color_estimation', default_value='false',
        description='Attach an advisory HSV-based colour hint to each detection',
    )

    # ノードの定義
    yolo_node = Node(
        package=pkg_name,
        executable='yolo_node',
        name='yolo_detector',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'device': LaunchConfiguration('device'),
            'camera_topic': LaunchConfiguration('camera_topic'),
            'publish_detections': LaunchConfiguration('publish_detections'),
            'detections_topic': LaunchConfiguration('detections_topic'),
            'use_image_roi': LaunchConfiguration('use_image_roi'),
            'roi_x_min_ratio': LaunchConfiguration('roi_x_min_ratio'),
            'roi_x_max_ratio': LaunchConfiguration('roi_x_max_ratio'),
            'roi_y_min_ratio': LaunchConfiguration('roi_y_min_ratio'),
            'roi_y_max_ratio': LaunchConfiguration('roi_y_max_ratio'),
            'enable_color_estimation': LaunchConfiguration('enable_color_estimation'),
        }]
    )

    return LaunchDescription([
        model_path_arg,
        device_arg,
        camera_topic_arg,
        publish_detections_arg,
        detections_topic_arg,
        use_image_roi_arg,
        roi_x_min_ratio_arg,
        roi_x_max_ratio_arg,
        roi_y_min_ratio_arg,
        roi_y_max_ratio_arg,
        enable_color_estimation_arg,
        yolo_node
    ])
