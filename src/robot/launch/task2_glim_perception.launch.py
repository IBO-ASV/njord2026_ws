"""Task 2 GLIM perception entry point; no planner or actuation.
Run separately from any existing odom->base_link EKF and Task 2 stack.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
 defaults={'use_sim_time':'false','glim_imu_topic':'/livox/imu','glim_points_topic':'/livox/lidar','glim_acc_scale':'1.0','publish_livox_mount_tf':'false','perception_world_frame':'odom','corridor_enabled':'false'}
 args={key:LaunchConfiguration(key) for key in defaults}
 args.update({'ego_odom_source':'glim','start_planner':'false','enable_ship_tracking':'true'})
 return LaunchDescription([*[DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()],IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('robot'),'launch','task2_jetson_autonomy.launch.py'])),launch_arguments=args.items())])
