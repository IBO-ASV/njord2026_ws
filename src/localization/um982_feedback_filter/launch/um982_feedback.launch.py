from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def mode_is(name):
    return IfCondition(PythonExpression(["'", LaunchConfiguration('feedback_mode'), "' == '", name, "'"]))


def ekf_fusion_is(enabled):
    return IfCondition(PythonExpression([
        "'", LaunchConfiguration('feedback_mode'), "' == 'ekf' and '",
        LaunchConfiguration('enable_glim_imu_fusion'), "'.lower() ",
        "in ('true', '1', 'yes', 'on')" if enabled else "not in ('true', '1', 'yes', 'on')",
    ]))


def generate_launch_description():
    package_share = get_package_share_directory('um982_feedback_filter')
    raw_topic = LaunchConfiguration('raw_topic')
    output_topic = LaunchConfiguration('output_topic')
    window_node = Node(
        package='um982_feedback_filter', executable='window_feedback_node',
        name='um982_window_feedback', output='screen', condition=mode_is('window'),
        parameters=[{'input_topic': raw_topic, 'output_topic': output_topic}],
    )
    um982_ekf_node = Node(
        package='robot_localization', executable='ekf_node', name='um982_feedback_ekf',
        output='screen', condition=ekf_fusion_is(False),
        parameters=[package_share + '/config/um982_feedback_ekf.yaml'],
        remappings=[('odometry/feedback', raw_topic), ('odometry/filtered', output_topic)],
    )
    glim_imu_ekf_node = Node(
        package='robot_localization', executable='ekf_node', name='um982_feedback_ekf',
        output='screen', condition=ekf_fusion_is(True),
        parameters=[package_share + '/config/um982_glim_imu_ekf.yaml'],
        remappings=[('odometry/feedback', raw_topic), ('odometry/filtered', output_topic)],
    )
    return LaunchDescription([
        DeclareLaunchArgument('feedback_mode', default_value='window', choices=['window', 'ekf']),
        DeclareLaunchArgument('raw_topic', default_value='/odometry/feedback'),
        DeclareLaunchArgument('output_topic', default_value='/odometry/filtered/local'),
        DeclareLaunchArgument(
            'enable_glim_imu_fusion', default_value='false',
            description='Fuse Jetson GLIM odometry and Livox IMU with UM982 feedback.',
        ),
        window_node,
        um982_ekf_node,
        glim_imu_ekf_node,
    ])
