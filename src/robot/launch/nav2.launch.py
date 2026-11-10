import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


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


def _launch_nav2(context, pkg_robot):
    profile = LaunchConfiguration('profile').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    if not params_file:
        profile_files = {
            'task1': 'nav2_params_humble.yaml',
            'task2': 'nav2_params_task2_humble.yaml',
            'task3': 'nav2_params_task3_humble.yaml',
        }
        try:
            params_file = os.path.join(pkg_robot, 'config', profile_files[profile])
        except KeyError as error:
            raise RuntimeError(
                f"Unknown Nav2 profile {profile!r}; expected one of "
                f"{', '.join(sorted(profile_files))}"
            ) from error
    nav_to_pose_bt_xml = LaunchConfiguration('nav_to_pose_bt_xml').perform(context)
    nav_through_poses_bt_xml = LaunchConfiguration('nav_through_poses_bt_xml').perform(context)
    default_nav_through_poses_bt_xml = os.path.join(
        pkg_robot, 'config', 'navigate_through_poses_w_replanning_and_recovery.xml'
    )
    if profile == 'task3' and nav_through_poses_bt_xml == default_nav_through_poses_bt_xml:
        nav_through_poses_bt_xml = os.path.join(
            pkg_robot, 'config', 'navigate_through_poses_task3_w_replanning_and_recovery.xml'
        )

    replanning_frequency = _load_replanning_frequency(params_file)
    configured_nav_to_pose_bt_xml = _write_configured_bt_xml(
        nav_to_pose_bt_xml,
        replanning_frequency,
    )
    configured_nav_through_poses_bt_xml = _write_configured_bt_xml(
        nav_through_poses_bt_xml,
        replanning_frequency,
    )

    configured_params = RewrittenYaml(
        source_file=params_file,
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
            'Optional full path to Nav2 parameters. Leave empty to select the '
            'validated profile-specific file.'
        )
    )
    profile_arg = DeclareLaunchArgument(
        'profile',
        default_value='task1',
        choices=['task1', 'task2', 'task3'],
        description=(
            'Resident Nav2 profile. It is selected at role bringup and is never '
            'changed by Mission Manager while propulsion can be enabled.'
        )
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
        profile_arg,
        params_file_arg,
        nav_to_pose_bt_xml_arg,
        nav_through_poses_bt_xml_arg,
        enable_diagnostics_arg,
        enable_collision_monitor_arg,
        OpaqueFunction(function=_launch_nav2, args=[pkg_robot]),
    ])
