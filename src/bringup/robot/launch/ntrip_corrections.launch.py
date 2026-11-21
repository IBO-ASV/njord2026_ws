"""Standalone caster plus optional fixed-station relay; no sensors or actuators."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration as C, PathJoinSubstitution as P
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def generate_launch_description():
    defaults = {
        'enable_ntrip_caster': 'true',
        'enable_ntrip_relay': 'true',
        'ntrip_caster_config': P([FindPackageShare('ntripcaster'), 'config', 'ntripcaster.json']),
        'ntrip_upstream_host': 'geortk.jp',
        'ntrip_upstream_port': '2101',
        'ntrip_upstream_mountpoint': 'tamtam',
        'ntrip_upstream_user': '',
        'ntrip_upstream_password': '',
        'ntrip_max_distance_km': '50.0',
        'ntrip_send_gga': 'true',
        'ntrip_require_gga': 'false',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()],
        ExecuteProcess(cmd=[P([FindPackagePrefix('ntripcaster'), 'lib', 'ntripcaster', 'ntripcaster']),
                            C('ntrip_caster_config')], output='screen',
                       condition=IfCondition(C('enable_ntrip_caster'))),
        ExecuteProcess(cmd=[P([FindPackagePrefix('robot'), 'lib', 'robot', 'ntrip_relay.py']),
                            '--caster-config', C('ntrip_caster_config'),
                            '--upstream-host', C('ntrip_upstream_host'),
                            '--upstream-port', C('ntrip_upstream_port'),
                            '--upstream-mountpoint', C('ntrip_upstream_mountpoint'),
                            '--upstream-user', C('ntrip_upstream_user'),
                            '--upstream-password', C('ntrip_upstream_password'),
                            '--max-distance-km', C('ntrip_max_distance_km'),
                            '--send-gga', C('ntrip_send_gga'),
                            '--require-gga', C('ntrip_require_gga')],
                       output='screen', condition=IfCondition(C('enable_ntrip_relay'))),
    ])
