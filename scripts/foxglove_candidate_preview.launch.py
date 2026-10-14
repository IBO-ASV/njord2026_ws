from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
def generate_launch_description():
 return LaunchDescription([DeclareLaunchArgument('bag'),
  Node(package='foxglove_bridge',executable='foxglove_bridge',name='candidate_preview_bridge',parameters=[{'port':8766,'address':'100.115.34.103','use_sim_time':False,'topic_whitelist':['^/preview/.*$'],'capabilities':['connectionGraph'],'publish_client_count':True}],output='screen'),
  ExecuteProcess(cmd=['python3',str(Path(__file__).with_name('foxglove_candidate_loop.py')),LaunchConfiguration('bag')],output='screen')])
