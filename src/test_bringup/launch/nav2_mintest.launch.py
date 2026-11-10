import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    ros_distro = os.environ.get('ROS_DISTRO', 'humble')
    config_distro = ros_distro if ros_distro in ('humble', 'jazzy') else 'humble'
    default_params_file = os.path.join(
        get_package_share_directory('test_bringup'),
        'config',
        f'nav2_mintest_{config_distro}.yaml',
    )
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Nav2 minimum-test parameters. Defaults to the active ROS distro.',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'params_file': params_file,
                'use_sim_time': use_sim_time,
                'autostart': 'true',
            }.items()
        )
    ])
