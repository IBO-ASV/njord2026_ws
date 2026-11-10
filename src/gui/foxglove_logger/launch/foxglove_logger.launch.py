from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    topic_arguments = {
        'cell_voltages_topic': '/bms/cell_voltages',
        'temperature_topic': '/bms/temperature_c',
        'buoy_topic': '/buoy_detections_3d',
        'fix_topic': '/sensor/vehicle_gnss/fix/raw',
        'heading_topic': '/sensor/vehicle_gnss/compass/raw',
        'odometry_topic': '/odometry/filtered/global',
        'plan_topic': '/plan',
        'ground_speed_topic': '/gui/ground_speed_mps',
        'control_status_topic': '/system/control_status',
        'control_state_topic': '/control/state',
        'publish_rate_hz': '1.0',
    }
    actions = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in topic_arguments.items()
    ]
    actions.append(Node(
        package='foxglove_logger', executable='foxglove_logger_node',
        name='foxglove_logger', output='screen',
        parameters=[{name: LaunchConfiguration(name) for name in topic_arguments}],
    ))
    return LaunchDescription(actions)
