from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    config = LaunchConfiguration('config')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument(
            'config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('alert_lamp'), 'config', 'alert_lamp.yaml']
            ),
        ),
        DeclareLaunchArgument(
            'require_rtk_fix', default_value='false',
            description='Deprecated compatibility argument; use control_policy.yaml instead.',
        ),
        Node(package='alert_lamp', executable='alert_lamp_manager_node',
             namespace=namespace, output='screen', parameters=[config]),
        Node(package='alert_lamp', executable='alert_lamp_driver_node',
             namespace=namespace, output='screen', parameters=[config]),
    ])
