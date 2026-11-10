"""Isolated recorded LiDAR detection + camera preview, no control nodes."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
 here=Path(__file__).parent
 params=str(Path(get_package_share_directory('task2_perception'))/'config/task2_params.yaml')
 return LaunchDescription([
  DeclareLaunchArgument('bag'),
  IncludeLaunchDescription(PythonLaunchDescriptionSource(str(here/'foxglove_candidate_preview.launch.py')),launch_arguments={'bag':LaunchConfiguration('bag'),'player':str(here/'foxglove_sensor_bag_loop.py')}.items()),
  Node(package='tf2_ros',executable='static_transform_publisher',name='preview_livox_mount',arguments=['--x','0.5','--y','0','--z','0.8','--roll','3.141592653589793','--pitch','0','--yaw','0','--frame-id','base_link','--child-frame-id','livox_frame']),
  Node(package='task2_perception',executable='task2_cloud_filter_node',name='task2_cloud_filter',parameters=[params,{'input_topic':'/preview/points','output_topic':'/preview/filtered_points','publish_self_marker':False}],output='screen'),
  Node(package='pcl_preprocessing',executable='preprocessing_node',name='preprocessing_node',parameters=[params],remappings=[('input/pointcloud','/preview/filtered_points'),('output/pointcloud_preprocessed','/pcl/preprocessed')]),
  Node(package='pcl_segmentation',executable='ground_remover_node',name='ground_remover_node',parameters=[params],remappings=[('input/pointcloud','/pcl/preprocessed'),('output/nonground','/pcl/nonground'),('output/ground','/pcl/ground')]),
  Node(package='pcl_segmentation',executable='cluster_node',name='cluster_node',parameters=[params],remappings=[('input/nonground','/pcl/nonground'),('output/cluster_markers','/preview/detections'),('output/cluster_observations','/preview/cluster_observations')])
 ])
