import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('sensor_sim_with_noise')
    config_file = os.path.join(pkg_share, 'config', 'config.yaml')
    fix_topic_arg = DeclareLaunchArgument(
        'fix_topic',
        default_value='/gps/fix',
        description='Output topic for the simulated NavSatFix message',
    )
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/wit/imu',
        description='Output topic for the simulated IMU message',
    )
    true_north_yaw_arg = DeclareLaunchArgument(
        'true_north_yaw_rad',
        default_value='1.5707963267948966',
        description='Geographical true-north direction expressed in the sim map frame (rad)',
    )

    gnss_sim_node = Node(
        package='sensor_sim_with_noise',
        executable='gnss_noise_simulator',
        name='gnss_noise_simulator',
        parameters=[config_file, {
            'true_north_yaw_rad': ParameterValue(
                LaunchConfiguration('true_north_yaw_rad'), value_type=float),
        }],
        remappings=[('/gps/fix', LaunchConfiguration('fix_topic'))],
        output='screen'
    )

    imu_sim_node = Node(
        package='sensor_sim_with_noise',
        executable='imu_noise_simulator',
        name='imu_noise_simulator',
        parameters=[config_file],
        remappings=[('/wit/imu', LaunchConfiguration('imu_topic'))],
        output='screen'
    )

    um982_odom_sim_node = Node(
        package='sensor_sim_with_noise',
        executable='um982_odometry_simulator',
        name='um982_odometry_simulator',
        parameters=[config_file],
        output='screen'
    )

    return LaunchDescription([
        fix_topic_arg,
        imu_topic_arg,
        true_north_yaw_arg,
        gnss_sim_node,
        imu_sim_node,
        um982_odom_sim_node
    ])
