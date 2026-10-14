"""Jetson half of Task 2: LiDAR perception, tracking and MPPI planning."""

import tempfile,yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import SetParameter


def _include(package, launch_file, arguments):
    return GroupAction(scoped=True, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare(package), "launch", launch_file])),
        launch_arguments=arguments.items(),
    )])


def _task2_parameters(context):
    # Explicit node-scoped YAML entries can override wildcard launch parameters.
    # Merge overrides into those same entries, keeping the source YAML read-only.
    data=yaml.safe_load((Path(get_package_share_directory('task2_perception'))/'config/task2_params.yaml').read_text())
    get=lambda key: LaunchConfiguration(key).perform(context)
    odom='/task2/odometry/local' if get('ego_odom_source')=='glim' else get('own_odom_topic')
    world=get('perception_world_frame')
    data['opponent_selector']['ros__parameters'].update(ego_odom_topic=odom,map_frame=world,motion_filter_mode='straight_line',corridor_enabled=get('corridor_enabled').lower()=='true')
    data['task2_waypoint_map_frame_relay']['ros__parameters']['target_frame']=world
    with tempfile.NamedTemporaryFile(mode='w',suffix='.yaml',prefix='task2_params_',delete=False) as stream:
        yaml.safe_dump(data,stream);name=stream.name
    return [SetLaunchConfiguration('task2_params_file',name)]

def generate_launch_description():
    odom = PythonExpression(["'/task2/odometry/local' if '", LaunchConfiguration("ego_odom_source"), "' == 'glim' else '", LaunchConfiguration("own_odom_topic"), "'"])
    localization = _include("robot", "task2_glim_localization.launch.py", {
        key: LaunchConfiguration(key) for key in ["use_sim_time", "glim_imu_topic", "glim_points_topic", "glim_acc_scale", "publish_livox_mount_tf"]
    })
    tracking = LaunchConfiguration("enable_ship_tracking")
    params = LaunchConfiguration("task2_params_file")
    perception = _include("task2_perception", "task2_perception.launch.py", {
        "params_file": params,
        "enable_cloud_filter": "true", "enable_opponent_selector": tracking,
        # Task 2 uses LiDAR only for other-vessel tracking.  Do not start the
        # optional buoy selector or buoy-based EKF correction pipeline.
        "enable_buoy_selector": "false",
        "publish_self_marker": "true", "ego_odom_topic": odom,
        "motion_filter_mode": "straight_line",
        "map_frame": LaunchConfiguration("perception_world_frame"),
        "corridor_enabled": LaunchConfiguration("corridor_enabled"),
        "use_sim_time": LaunchConfiguration("use_sim_time"),
    })
    preprocessing = _include("pcl_preprocessing", "preprocessing.launch.py", {
        "config_file": params, "input_topic": "/task2/points_filtered", "output_topic": "/pcl/preprocessed",
    })
    segmentation = _include("pcl_segmentation", "segmentation.launch.py", {
        "config_file": params, "use_color": "false",
    })
    ship_tracking = _include("ship_tracking", "tracker.launch.py", {
        "config_file": params, "ego_odom_topic": odom,
    })
    planner = _include("asv_trajectory_planner", "planner_real.launch.py", {
        "own_odom_topic": odom, "mission_gate_required": "false",
        "start_follow_path_client": "false", "start_waypoint_pose_publisher": "false",
    })
    return LaunchDescription([
        DeclareLaunchArgument("own_odom_topic", default_value="/odometry/filtered/local"),
        DeclareLaunchArgument("enable_ship_tracking", default_value="true"),
        DeclareLaunchArgument("ego_odom_source", default_value="external", choices=["external", "glim"]),
        DeclareLaunchArgument("start_planner", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("glim_imu_topic", default_value="/livox/imu"),
        DeclareLaunchArgument("glim_points_topic", default_value="/livox/lidar"),
        DeclareLaunchArgument("glim_acc_scale", default_value="1.0"),
        DeclareLaunchArgument("publish_livox_mount_tf", default_value="false"),
        DeclareLaunchArgument("perception_world_frame", default_value="map"),
        DeclareLaunchArgument("corridor_enabled", default_value="true"),
        OpaqueFunction(function=_task2_parameters),
        SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim_time")),
        GroupAction(condition=IfCondition(PythonExpression(["'", LaunchConfiguration("ego_odom_source"), "' == 'glim'"])), actions=[localization]),
        perception,
        GroupAction(condition=IfCondition(tracking), actions=[preprocessing, segmentation, ship_tracking]),
        GroupAction(condition=IfCondition(LaunchConfiguration("start_planner")), actions=[planner]),
    ])
