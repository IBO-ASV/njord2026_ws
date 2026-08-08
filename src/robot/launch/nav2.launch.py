import os
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml

from mission_manager.nav2_profiles import Nav2ProfileCatalog


def _load_replanning_frequency(params_file):
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f) or {}
    bt_params = params.get('bt_navigator', {}).get('ros__parameters', {})
    return float(bt_params.get('replanning_frequency', 1.0))


def _write_configured_bt_xml(source_file, replanning_frequency):
    with open(source_file, 'r') as f:
        xml = f.read()
    xml = xml.replace('__REPLANNING_FREQUENCY__', f'{replanning_frequency:.6g}')
    tmp = tempfile.NamedTemporaryFile(
        mode='w',
        prefix='nav2_bt_',
        suffix='.xml',
        delete=False,
    )
    with tmp:
        tmp.write(xml)
    return tmp.name


def _write_resolved_profile(catalog_file, active_profile, ros_distro):
    """Resolve and validate the selected startup profile before Nav2 starts."""
    catalog = Nav2ProfileCatalog.from_file(Path(catalog_file))
    params = catalog.resolve(active_profile, ros_distro)
    tmp = tempfile.NamedTemporaryFile(
        mode='w',
        prefix=f'nav2_{active_profile}_{ros_distro}_',
        suffix='.yaml',
        delete=False,
    )
    with tmp:
        yaml.safe_dump(params, tmp, sort_keys=False)
    return tmp.name


def _launch_nav2(context, pkg_robot):
    params_file = LaunchConfiguration('params_file').perform(context)
    active_profile = LaunchConfiguration('active_nav2_profile').perform(context)
    profile_catalog_file = LaunchConfiguration('nav2_profile_catalog_file').perform(
        context
    )
    nav_to_pose_bt_xml = LaunchConfiguration('nav_to_pose_bt_xml').perform(context)
    nav_through_poses_bt_xml = LaunchConfiguration('nav_through_poses_bt_xml').perform(context)

    if params_file:
        # Compatibility-only path used by the legacy comparison launches. The
        # persistent miniPC bringup never supplies this override.
        startup_params_file = params_file
        startup_description = f'explicit compatibility params {params_file}'
    else:
        ros_distro = os.environ.get('ROS_DISTRO', '')
        if not ros_distro:
            raise RuntimeError('ROS_DISTRO is required to resolve a Nav2 startup profile')
        startup_params_file = _write_resolved_profile(
            profile_catalog_file,
            active_profile,
            ros_distro,
        )
        startup_description = (
            f'catalog profile {active_profile} for ROS_DISTRO={ros_distro}'
        )

    replanning_frequency = _load_replanning_frequency(startup_params_file)
    configured_nav_to_pose_bt_xml = _write_configured_bt_xml(
        nav_to_pose_bt_xml,
        replanning_frequency,
    )
    configured_nav_through_poses_bt_xml = _write_configured_bt_xml(
        nav_through_poses_bt_xml,
        replanning_frequency,
    )

    configured_params = RewrittenYaml(
        source_file=startup_params_file,
        root_key=None,
        param_rewrites={
            'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml':
                configured_nav_to_pose_bt_xml,
            'bt_navigator.ros__parameters.default_nav_through_poses_bt_xml':
                configured_nav_through_poses_bt_xml,
            # Collision Monitor remains in Nav2's standard velocity pipeline. Disable
            # both its worker and LiDAR source together: disabling only the worker
            # still makes an absent LiDAR source trigger its fail-safe stop.
            'collision_monitor.ros__parameters.enabled':
                LaunchConfiguration('enable_collision_monitor'),
            'collision_monitor.ros__parameters.livox.enabled':
                LaunchConfiguration('enable_collision_monitor'),
        },
        convert_types=True,
    )

    diagnostics_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot, 'launch', 'diagnostics.launch.py')
        ),
        launch_arguments={'profile': 'nav2'}.items(),
        condition=IfCondition(LaunchConfiguration('enable_diagnostics')),
    )

    return [
        LogInfo(msg=f'Starting Nav2 with {startup_description}'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_robot, 'launch', 'navigation.launch.py')
            ),
            launch_arguments={
                'params_file': configured_params,
                'use_sim_time': 'false',
                'autostart': 'true'
            }.items()
        ),
        diagnostics_launch,
    ]


def generate_launch_description():
    pkg_robot = get_package_share_directory('robot')
    pkg_mission_manager = get_package_share_directory('mission_manager')

    default_profile_catalog_file = os.path.join(
        pkg_mission_manager,
        'config',
        'nav2_profiles',
        'profiles.yaml',
    )
    default_nav_to_pose_bt_xml = os.path.join(
        pkg_robot, 'config', 'navigate_to_pose_w_replanning_and_recovery.xml'
    )
    default_nav_through_poses_bt_xml = os.path.join(
        pkg_robot, 'config', 'navigate_through_poses_w_replanning_and_recovery.xml'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value='',
        description=(
            'Compatibility only: explicit full Nav2 parameter file. When empty, '
            'active_nav2_profile is resolved from the validated profile catalog.'
        ),
    )
    active_nav2_profile_arg = DeclareLaunchArgument(
        'active_nav2_profile',
        default_value='task1',
        choices=['task1', 'task2', 'task3'],
        description='Startup-only resident profile resolved before Nav2 is launched.',
    )
    nav2_profile_catalog_file_arg = DeclareLaunchArgument(
        'nav2_profile_catalog_file',
        default_value=default_profile_catalog_file,
        description='Validated startup-only Nav2 profile catalog.',
    )
    nav_to_pose_bt_xml_arg = DeclareLaunchArgument(
        'nav_to_pose_bt_xml',
        default_value=default_nav_to_pose_bt_xml,
        description='Full path to the NavigateToPose behavior tree XML'
    )
    nav_through_poses_bt_xml_arg = DeclareLaunchArgument(
        'nav_through_poses_bt_xml',
        default_value=default_nav_through_poses_bt_xml,
        description='Full path to the NavigateThroughPoses behavior tree XML'
    )
    enable_diagnostics_arg = DeclareLaunchArgument(
        'enable_diagnostics',
        default_value='true',
        description='Launch generic topic heartbeat diagnostics for Nav2 topics'
    )
    enable_collision_monitor_arg = DeclareLaunchArgument(
        'enable_collision_monitor',
        default_value='true',
        description=(
            'Enable Collision Monitor obstacle processing. When false, its LiDAR '
            'source and actions are disabled while it remains as a cmd_vel passthrough.'
        ),
    )

    return LaunchDescription([
        params_file_arg,
        active_nav2_profile_arg,
        nav2_profile_catalog_file_arg,
        nav_to_pose_bt_xml_arg,
        nav_through_poses_bt_xml_arg,
        enable_diagnostics_arg,
        enable_collision_monitor_arg,
        OpaqueFunction(function=_launch_nav2, args=[pkg_robot]),
    ])
