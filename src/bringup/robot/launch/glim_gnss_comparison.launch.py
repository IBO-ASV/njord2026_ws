"""Offline comparison only: prepared ENU GLIM plus GNSS, no TF or controls."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,ExecuteProcess,TimerAction
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
 root=LaunchConfiguration('result_dir')
 params={'use_sim_time':True,'frequency':20.,'sensor_timeout':.5,'two_d_mode':True,'publish_tf':False,
  'map_frame':'map','odom_frame':'odom','base_link_frame':'base_link','world_frame':'map',
  'odom0':'/comparison/glim_enu','odom0_config':[False,False,False,False,False,True,True,True,False,False,False,True,False,False,False],
  'odom0_queue_size':100,'odom0_differential':False,'odom0_relative':False,
  'odom1':'/comparison/gnss_base','odom1_config':[True,True,False,False,False,False,False,False,False,False,False,False,False,False,False],
  'odom1_queue_size':100,'odom1_differential':False,'odom1_relative':False}
 return LaunchDescription([DeclareLaunchArgument('result_dir'),
  Node(package='robot_localization',executable='ekf_node',name='offline_gnss_glim_ekf',parameters=[params],remappings=[('odometry/filtered','/comparison/fused')],output='screen'),
  Node(package='robot_localization',executable='ekf_node',name='offline_gnss_glim_conservative',parameters=[dict(params,odom1='/comparison/gnss_base_conservative')],remappings=[('odometry/filtered','/comparison/fused_conservative')],output='screen'),
  ExecuteProcess(cmd=['ros2','bag','record','-s','mcap','-o',PathJoinSubstitution([root,'fusion_output_bag']),'/comparison/fused','/comparison/fused_conservative'],output='screen'),
  TimerAction(period=3.,actions=[ExecuteProcess(cmd=['ros2','bag','play',PathJoinSubstitution([root,'fusion_input_bag']),'--clock','100','--rate','3','--disable-keyboard-controls'],output='screen')])])
