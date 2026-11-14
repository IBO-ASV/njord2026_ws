"""Isolated GLIM odometry + perception check; no control or GNSS fusion."""
import json
import re
import shutil
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def setup(context):
    share = Path(get_package_share_directory('robot'))
    output = Path(LaunchConfiguration('output_dir').perform(context))
    config = output / 'glim_config'
    shutil.copytree(share / 'config/glim_config_headless', config, dirs_exist_ok=True)
    path = config / 'config_ros.json'
    text = re.sub(r'/\*.*?\*/|//[^\n]*', '', path.read_text(), flags=re.S)
    data = json.loads(text)
    data['glim_ros']['map_frame_id'] = 'map'
    path.write_text(json.dumps(data, indent=2))
    params = str(Path(get_package_share_directory('task2_perception')) / 'config/task2_params.yaml')
    pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(get_package_share_directory('ship_perception_bringup')) / 'launch/classical_pipeline.launch.py')),
        launch_arguments={'lidar_topic':'/task2/points_filtered','ego_odom_topic':'/odometry/filtered/local',
                          'preprocessing_config_file':params,'segmentation_config_file':params,'tracker_config_file':params}.items())
    return [
        SetParameter(name='use_sim_time', value=True),
        Node(package='tf2_ros', executable='static_transform_publisher', name='bag_livox_tf',
             arguments=['--x','0.5','--y','0','--z','0.8','--roll','3.141592653589793','--pitch','0','--yaw','0','--frame-id','base_link','--child-frame-id','livox_frame']),
        Node(package='tf2_ros', executable='static_transform_publisher', name='offline_map_tf',
             arguments=['--frame-id','map','--child-frame-id','odom']),
        Node(package='glim_ros', executable='glim_rosnode', name='glim_ros', output='screen',
             parameters=[{'config_path':str(config),'use_sim_time':True}],
             remappings=[('/livox/imu_raw','/livox/imu'),('/tf','/glim_private/tf')]),
        Node(package='task2_perception', executable='glim_base_odom_adapter', name='glim_base_odom_adapter',
             parameters=[{'use_sim_time':True}], output='screen'),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node_glim_local', output='screen',
             parameters=[str(share/'config/ekf_glim_local.yaml'),{'use_sim_time':True}],
             remappings=[('/odom','/glim_base/odom'),('odometry/filtered','/odometry/filtered/local')]),
        Node(package='task2_perception', executable='task2_cloud_filter_node', name='task2_cloud_filter', output='screen',
             parameters=[params,{'use_sim_time':True}]),
        pipeline,
        ExecuteProcess(cmd=['ros2','bag','record','-s','mcap','-o',str(output/'output_bag'),
                            '/glim_ros/odom','/glim_base/odom','/odometry/filtered/local','/tracked_objects','/task2/points_filtered','/tf','/tf_static','/sensor/vehicle_gnss/fix/raw'], output='screen'),
        TimerAction(period=8.0,actions=[ExecuteProcess(cmd=['ros2','bag','play',LaunchConfiguration('bag_path'),
            '--start-offset',LaunchConfiguration('start_offset'),'--playback-duration',LaunchConfiguration('duration'),
            '--clock','100','--disable-keyboard-controls','--topics','/livox/lidar','/livox/imu','/sensor/vehicle_gnss/fix/raw'],output='screen')]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag_path'),DeclareLaunchArgument('output_dir'),
        DeclareLaunchArgument('start_offset',default_value='0'),DeclareLaunchArgument('duration',default_value='120'),
        OpaqueFunction(function=setup)])
