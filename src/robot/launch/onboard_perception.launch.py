"""AGX task perception, shared continuous odom supplied by onboard_localization."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as C, PathJoinSubstitution as P, PythonExpression
from launch_ros.actions import Node, SetParameter, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    task1=PythonExpression(["'",C('task'),"' == 'task1'"])
    task2=PythonExpression(["'",C('task'),"' == 'task2'"])
    task1_safety_points = Node(
        package="task2_perception",
        executable="task2_cloud_filter_node",
        name="task1_safety_points",
        output="screen",
        parameters=[{
            "input_topic": "/zed2i/points",
            "output_topic": "/task1/safety_points",
            "visual_output_topic": "/task1/safety_points_visual",
            "publish_visual_z_mirror": False,
            "output_frame": "base_link",
            "min_range_m": 0.5,
            "max_range_m": 8.0,
            "min_valid_input_points": 0,
            "publish_empty_on_invalid_input": True,
            "voxel_leaf_size_m": 0.25,
            "accumulation_frames": 1,
            "process_rate_hz": 5.0,
            "waterline_z_m": 0.0,
            "water_remove_min_z_m": -0.3,
            "water_remove_max_z_m": 0.15,
            "use_water_plane_ransac": False,
            "object_min_z_m": -0.5,
            "object_max_z_m": 2.0,
            "publish_self_marker": False,
            "publish_debug": False,
        }],
        condition=IfCondition(task1),
    )

    task1_default_buoy_roi = Node(
        package="task1_buoy_roi",
        executable="task1_buoy_roi",
        name="task1_buoy_roi",
        output="screen",
        condition=IfCondition(PythonExpression(["'",C('task'),"' == 'task1' and '",C('enable_surveyed_buoy_roi'),"' == 'true'"])),
    )

    return LaunchDescription([
        DeclareLaunchArgument('enable_surveyed_buoy_roi',default_value='false'),
        DeclareLaunchArgument('task',default_value='task1',choices=['task1','task2']),
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        DeclareLaunchArgument('corridor_enabled',default_value='true'),
        DeclareLaunchArgument('start_planner',default_value='false'),
        SetParameter(name='use_sim_time',value=C('use_sim_time')),
        task1_safety_points,task1_default_buoy_roi,
        ComposableNodeContainer(name='task1_buoy_detector',namespace='',package='rclcpp_components',executable='component_container',condition=IfCondition(task1),composable_node_descriptions=[ComposableNode(package='pcl_det',plugin='pcl_det::PclBuoyDetectionNode',name='pcl_buoy_det_node',parameters=[{'input_topic':'/livox/lidar','roi_topic':'/buoy_roi','output_topic':'/buoy_detections','frame_id':'base_link','use_sim_time':C('use_sim_time')}])]),
        GroupAction(condition=IfCondition(task2),actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(P([FindPackageShare('robot'),'launch','task2_jetson_autonomy.launch.py'])),
            launch_arguments={'ego_odom_source':'external','own_odom_topic':'/odometry/filtered/local','start_planner':C('start_planner'),'use_sim_time':C('use_sim_time'),'perception_world_frame':'odom','corridor_enabled':C('corridor_enabled')}.items())])
    ])
