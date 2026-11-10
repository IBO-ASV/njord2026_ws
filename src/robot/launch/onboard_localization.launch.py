"""Shared Task1/Task2 continuous GLIM odom and optional GNSS map correction.
/fromLL remains in odom for existing Task1/Task2 route consumers.
The separate global navsat projector uses map, with private services.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as C, PathJoinSubstitution as P
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def include(name,args):
 return IncludeLaunchDescription(PythonLaunchDescriptionSource(P([FindPackageShare('robot'),'launch',name])),launch_arguments=args.items())
def generate_launch_description():
 sim=C('use_sim_time')
 common={'use_sim_time':sim,'frequency':10.0,'zero_altitude':True,'broadcast_utm_transform':False,'wait_for_datum':False,'use_odometry_yaw':False}
 heading=('imu','/localization/gnss_heading_imu');fix=('gps/fix','/sensor/vehicle_gnss/fix/raw')
 return LaunchDescription([
  DeclareLaunchArgument('use_sim_time',default_value='false'),
  DeclareLaunchArgument('enable_gnss_map',default_value='true'),
  DeclareLaunchArgument('glim_acc_scale',default_value='1.0'),
  include('localization.launch.py',{'use_sim_time':sim,'enable_glim':'false','enable_local_ekf':'false','enable_global_ekf':'false','enable_navsat_transform':'false','enable_diagnostics':'false'}),
  include('task2_glim_localization.launch.py',{'use_sim_time':sim,'glim_acc_scale':C('glim_acc_scale'),'local_odom_topic':'/odometry/filtered/local'}),
  GroupAction(condition=IfCondition(C('enable_gnss_map')),actions=[
   Node(package='robot',executable='gnss_heading_imu',parameters=[{'use_sim_time':sim}]),
   Node(package='robot_localization',executable='ekf_node',name='ekf_onboard_map',parameters=[P([FindPackageShare('robot'),'config','ekf_onboard_map.yaml']),{'use_sim_time':sim}],remappings=[('odometry/filtered','/odometry/filtered/global')]),
   Node(package='robot_localization',executable='navsat_transform_node',name='navsat_map',parameters=[common],remappings=[heading,fix,('odometry/filtered','/odometry/filtered/global'),('odometry/gps','/localization/gnss_map'),('fromLL','/localization/map/fromLL'),('toLL','/localization/map/toLL'),('datum','/localization/map/datum'),('setUTMZone','/localization/map/setUTMZone')]),
   Node(package='robot_localization',executable='navsat_transform_node',name='navsat_routes_odom',parameters=[common],remappings=[heading,fix,('odometry/filtered','/odometry/filtered/local'),('odometry/gps','/localization/gnss_odom_projection')]),
  ])
 ])
