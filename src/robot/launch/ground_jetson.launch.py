"""Ground station on the Jetson, paired with the AGX vessel."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    runtime_parameter_sync = Node(
        package='robot', executable='runtime_parameter_sync.py',
        name='runtime_parameter_sync', output='screen',
        parameters=[{
            'role': 'groundpc',
            'config_file': PathJoinSubstitution([
                FindPackageShare('robot'), 'runtime_params', 'runtime_parameters.yaml'
            ]),
            'publish_updates': True,
        }],
    )

    return LaunchDescription([
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
        SetEnvironmentVariable("CYCLONEDDS_URI", PathJoinSubstitution([FindPackageShare("robot"), "config", "cyclonedds_agx.xml"])),
        SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST"),
        SetEnvironmentVariable('ROS_DOMAIN_ID','6'),
        runtime_parameter_sync,
        DeclareLaunchArgument('enable_critical_link',default_value='true'),
        GroupAction(scoped=True,actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('robot'),'launch','ground_pc.launch.py'])),
            launch_arguments={'enable_zenoh_bridge':'false','enable_critical_link':'false',
                              'enable_foxglove_bridge':'false'}.items())]),
        Node(package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge',
             parameters=[{'port':8765}], output='screen'),
        ExecuteProcess(cmd=['env','-u','ROS_DOMAIN_ID','zenoh-bridge-ros2dds','-c',
            PathJoinSubstitution([FindPackageShare('robot'),'config','zenoh','bridge_ground_jetson.json5'])], output='screen'),
        Node(package='critical_link',executable='critical_link_sender',name='critical_link_sender',
             output='screen',condition=IfCondition(LaunchConfiguration('enable_critical_link')),
             parameters=[{'udp_paths':['tailscale|100.115.34.103|100.73.137.72|45101'],
                          'serial_device':'','key_file':'/etc/njord/critical_link.key'}]),
    ])
