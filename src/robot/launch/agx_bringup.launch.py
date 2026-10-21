"""AGX vessel bringup for one onboard PC and a separate ground Jetson."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def include(package, filename, arguments=None, condition=None):
    return GroupAction(scoped=True, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare(package), 'launch', filename])),
        launch_arguments=(arguments or {}).items(), condition=condition)])


def generate_launch_description():
    cfg = LaunchConfiguration
    um982 = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
    camera = '/dev/v4l/by-id/usb-Technologies__Inc._ZED_2i_OV0001-video-index0'
    defaults = {
        'task': 'task1', 'enable_glim': 'true', 'enable_task_perception': 'true',
        'use_sim_time': 'false', 'glim_acc_scale': '1.0',
        'enable_mid360': 'false',  # opt in after checking the sensor network
        'lidar_model': 'mid360s',
        'enable_zed2i': str(Path(camera).exists()).lower(),
        'enable_um982': str(Path(um982).exists()).lower(),
        'um982_port': um982,
        # BMS master XIAO ESP32-C3 on this AGX.  Keep this explicit in the
        # AGX profile rather than relying on the generic miniPC default.
        'bms_serial_port': ('/dev/serial/by-id/'
                            'usb-Espressif_USB_JTAG_serial_debug_unit_'
                            'E0:72:A1:1D:4B:14-if00'),
        'enable_thruster': 'false',  # hardware outputs require explicit opt in
        'enable_bms': 'false', 'enable_alert_lamp': 'false',
        'enable_back_cam': 'false', 'enable_nav2': 'false',
        'enable_critical_link': 'true',
        'ground_video_host': '100.115.34.103',
        'camera_resolution':'VGA', 'camera_fps':'15',
        'enable_gpu_perception': PythonExpression(["'true' if '",cfg('task'),"' == 'task1' else 'false'"]),
    }
    core_args = {key: cfg(key) for key in (
        'enable_um982', 'um982_port', 'bms_serial_port', 'enable_thruster', 'enable_bms',
        'enable_alert_lamp', 'enable_back_cam', 'enable_nav2', 'use_sim_time')}
    core_args.update({'enable_zenoh_bridge': 'false', 'enable_critical_link': 'false',
                      'enable_back_cam_ground_video': 'false',
                      'enable_back_cam_jpeg_ground_video': 'false',
                      'enable_autonomy_supervisor': 'false', 'active_nav2_profile': cfg('task'),
                      'enable_localization': PythonExpression(["'false' if '",cfg('enable_glim'),"' == 'true' else 'true'"]),
                      'enable_feedback_filter': PythonExpression(["'false' if '",cfg('enable_glim'),"' == 'true' else 'true'"])})
    runtime_parameter_sync = Node(
        package='robot', executable='runtime_parameter_sync.py',
        name='runtime_parameter_sync', output='screen',
        parameters=[{'role': 'minipc', 'publish_updates': False}],
    )
    return LaunchDescription([
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
        SetEnvironmentVariable("CYCLONEDDS_URI", PathJoinSubstitution([FindPackageShare("robot"), "config", "cyclonedds_agx.xml"])),
        SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST"),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '5'),
        runtime_parameter_sync,
        *[DeclareLaunchArgument(key, default_value=value) for key,value in defaults.items()],
        include('robot', 'minipc_bringup.launch.py', core_args),
        Node(package='robot', executable='agx_rosbag_record_manager', name='agx_rosbag_record_manager', output='screen'),
        include('robot','onboard_localization.launch.py',{'use_sim_time':cfg('use_sim_time'),'glim_acc_scale':cfg('glim_acc_scale'),'enable_gnss_map':cfg('enable_um982')},IfCondition(cfg('enable_glim'))),
        include('robot','onboard_perception.launch.py',{'task':cfg('task'),'use_sim_time':cfg('use_sim_time'),'start_planner':cfg('enable_nav2')},IfCondition(cfg('enable_task_perception'))),
        include('robot', 'lidar.launch.py', {'lidar_model': cfg('lidar_model'), 'enable_glim': 'false'}, IfCondition(cfg('enable_mid360'))),
        include('zed2i_driver', 'zed2i.launch.py', {
            'mode': 'sdk', 'enable_gpu_perception': cfg('enable_gpu_perception'),
            'params_file':PathJoinSubstitution([FindPackageShare('robot'),'config','zed2i_agx.yaml']),
            'engine_path': PathJoinSubstitution([FindPackageShare('robot'),'config','yolo_model','best.engine']),
            'enable_ground_video': 'true', 'ground_video_host': cfg('ground_video_host'),
            'ground_video_port': '5600', 'framerate': cfg('camera_fps'),
            'camera_resolution':cfg('camera_resolution'), 'ground_video_draw_detections':'true',
        }, IfCondition(cfg('enable_zed2i'))),
        include('robot', 'heartbeat.launch.py', {'role':'jetson'}),
        Node(package='livox_gui_telemetry', executable='livox_gui_downsampler',
             name='livox_gui_downsampler', output='screen', condition=IfCondition(cfg('enable_mid360'))),
        ExecuteProcess(cmd=['env','-u','ROS_DOMAIN_ID',str(Path.home()/'.local/bin/zenoh-bridge-ros2dds'),'-c',
            PathJoinSubstitution([FindPackageShare('robot'),'config','zenoh','bridge_agx.json5'])],output='screen'),
        Node(package='critical_link', executable='critical_link_receiver', name='critical_link_receiver',
             output='screen', condition=IfCondition(cfg('enable_critical_link')),
             parameters=[{'udp_paths':['tailscale|100.73.137.72|45101'],
                          'serial_device':'', 'key_file':'/etc/njord/critical_link.key',
                          'max_frame_age_ms':2000, 'future_tolerance_ms':1000}]),
    ])
