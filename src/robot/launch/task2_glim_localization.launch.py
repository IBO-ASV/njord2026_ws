"""GLIM -> base velocity/lever correction -> Task 2 local odom.
This launch must be the sole odom->base_link TF owner. No sensor drivers.
"""
import json,re,tempfile
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def setup(context):
 share=Path(get_package_share_directory('robot'))
 source=share/'config/glim_config_headless'
 config=Path(tempfile.mkdtemp(prefix='task2_glim_'))
 # Resolve config symlinks into a private runtime copy; never modify shared configs.
 for p in source.iterdir():
  if p.is_file(): (config/p.name).write_bytes(p.read_bytes())
 p=config/'config_ros.json'
 data=json.loads(re.sub(r'/\*.*?\*/|//[^\n]*','',p.read_text(),flags=re.S))
 ros=data['glim_ros'];ros['imu_topic']=LaunchConfiguration('glim_imu_topic').perform(context)
 ros['points_topic']=LaunchConfiguration('glim_points_topic').perform(context)
 ros['acc_scale']=float(LaunchConfiguration('glim_acc_scale').perform(context))
 ros['map_frame_id']='map';ros['enable_local_mapping']=False;ros['enable_global_mapping']=False
 p.write_text(json.dumps(data,indent=2))
 sim=LaunchConfiguration('use_sim_time')
 return [
  Node(package='glim_ros',executable='glim_rosnode',name='task2_glim',parameters=[{'config_path':str(config),'use_sim_time':sim}],remappings=[('/task2_glim/odom','/task2/glim/raw_odom'),('/tf','/task2/glim/private_tf'),('/tf_static','/task2/glim/private_tf_static')],output='screen'),
  Node(package='task2_perception',executable='glim_base_odom_adapter',name='task2_glim_adapter',parameters=[{'use_sim_time':sim}],remappings=[('/glim_ros/odom','/task2/glim/raw_odom'),('/glim_base/odom','/task2/glim/base_odom')],output='screen'),
  Node(package='robot_localization',executable='ekf_node',name='ekf_filter_node_glim_local',parameters=[str(share/'config/ekf_glim_local.yaml'),{'use_sim_time':sim}],remappings=[('/odom','/task2/glim/base_odom'),('odometry/filtered','/task2/odometry/local')],output='screen'),
  Node(package='tf2_ros',executable='static_transform_publisher',name='task2_bag_livox_mount',arguments=['--x','0.5','--y','0','--z','0.8','--roll','3.141592653589793','--pitch','0','--yaw','0','--frame-id','base_link','--child-frame-id','livox_frame'],condition=IfCondition(LaunchConfiguration('publish_livox_mount_tf')))
 ]

def generate_launch_description():
 return LaunchDescription([
  DeclareLaunchArgument('use_sim_time',default_value='false'),
  DeclareLaunchArgument('glim_imu_topic',default_value='/livox/imu'),
  DeclareLaunchArgument('glim_points_topic',default_value='/livox/lidar'),
  DeclareLaunchArgument('glim_acc_scale',default_value='1.0',description='1 for SI IMU; 9.80665 for historical g-unit bags'),
  DeclareLaunchArgument('publish_livox_mount_tf',default_value='false'),
  OpaqueFunction(function=setup)])
