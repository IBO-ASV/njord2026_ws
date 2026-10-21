"""Ground station on the Jetson, paired with the AGX vessel."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    backend = LaunchConfiguration('communication_backend')
    use_zenoh = PythonExpression(["'", backend, "' == 'zenoh'"])
    return LaunchDescription([
        DeclareLaunchArgument('communication_backend', default_value=EnvironmentVariable(
            'NJORD_AGX_TRANSPORT', default_value='fastrtps'),
                              description='ROS transport: direct Fast DDS or legacy Zenoh bridge.'),
        DeclareLaunchArgument('fastdds_local_ip', default_value='100.115.34.103'),
        DeclareLaunchArgument('fastdds_peer_ip', default_value='100.73.137.72'),
        SetEnvironmentVariable("RMW_IMPLEMENTATION", PythonExpression([
            "'rmw_fastrtps_cpp' if '", backend, "' == 'fastrtps' else 'rmw_cyclonedds_cpp'"
        ])),
        SetEnvironmentVariable("CYCLONEDDS_URI", PathJoinSubstitution([FindPackageShare("robot"), "config", "cyclonedds_agx.xml"])),
        SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", PathJoinSubstitution([
            FindPackageShare('robot'), 'config', 'dds', 'fastdds_agx_pair.xml'
        ])),
        SetEnvironmentVariable('NJORD_FASTDDS_LOCAL_IP', LaunchConfiguration('fastdds_local_ip')),
        SetEnvironmentVariable('NJORD_FASTDDS_PEER_IP', LaunchConfiguration('fastdds_peer_ip')),
        SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", PythonExpression([
            "'SUBNET' if '", backend, "' == 'fastrtps' else 'LOCALHOST'"
        ])),
        SetEnvironmentVariable('ROS_DOMAIN_ID', PythonExpression([
            "'126' if '", backend, "' == 'fastrtps' else '6'"
        ])),
        DeclareLaunchArgument('enable_critical_link',default_value='true'),
        GroupAction(scoped=True,actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('robot'),'launch','ground_pc.launch.py'])),
            launch_arguments={'enable_zenoh_bridge':'false','enable_critical_link':'false',
                              'enable_foxglove_bridge':'false'}.items())]),
        Node(package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge',
             parameters=[{'port':8765}], output='screen'),
        ExecuteProcess(cmd=['env','-u','ROS_DOMAIN_ID','zenoh-bridge-ros2dds','-c',
            PathJoinSubstitution([FindPackageShare('robot'),'config','zenoh','bridge_ground_jetson.json5'])],
            output='screen', condition=IfCondition(use_zenoh)),
        Node(package='critical_link',executable='critical_link_sender',name='critical_link_sender',
             output='screen',condition=IfCondition(LaunchConfiguration('enable_critical_link')),
             parameters=[{'udp_paths':['tailscale|100.115.34.103|100.73.137.72|45101'],
                          'serial_device':'','key_file':'/etc/njord/critical_link.key'}]),
    ])
