"""Minimal GLIM + UM982 localization with one TF owner per dynamic edge.

GLIM supplies continuous local odometry but its TF must remain private.  The
local EKF publishes odom -> base_link.  UM982 supplies a first-fix local ENU
pose labelled as map, and the global EKF publishes map -> odom.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_share = FindPackageShare("robot")
    use_sim_time = LaunchConfiguration("use_sim_time")

    um982_driver = GroupAction(
        condition=IfCondition(LaunchConfiguration("enable_um982_driver")),
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("um982_driver"), "launch", "um982.launch.py"]
                    )
                ),
                launch_arguments={
                    "uart_or_tcp": LaunchConfiguration("um982_protocol"),
                    "gnss_port": LaunchConfiguration("um982_port"),
                    "gnss_baudrate": LaunchConfiguration("um982_baudrate"),
                    "rtk_enable": LaunchConfiguration("enable_um982_rtk"),
                    "publish_feedback_odometry": "true",
                    "feedback_frame_id": "map",
                    "feedback_child_frame_id": "base_link",
                    "heading_frame_id": "map",
                }.items(),
            )
        ],
    )

    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution([robot_share, "urdf", "robot.urdf.xacro"]),
        ]
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": ParameterValue(robot_description, value_type=str),
            }
        ],
    )
    um982_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="um982_static_tf_pub",
        output="screen",
        arguments=[
            "--x", "0.0", "--y", "0.0", "--z", "0.0",
            "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
            "--frame-id", "base_link", "--child-frame-id", "um982_link",
        ],
    )

    local_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_glim_local",
        output="screen",
        parameters=[
            PathJoinSubstitution([robot_share, "config", "ekf_glim_local.yaml"]),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("/odom", LaunchConfiguration("glim_odom_topic")),
            ("odometry/filtered", "/odometry/filtered/local"),
        ],
    )
    global_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_um982_map",
        output="screen",
        parameters=[
            PathJoinSubstitution([robot_share, "config", "ekf_um982_map.yaml"]),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("/odometry/feedback", LaunchConfiguration("um982_feedback_topic")),
            ("odometry/filtered", "/odometry/filtered/global"),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("enable_um982_driver", default_value="true"),
            DeclareLaunchArgument("enable_um982_rtk", default_value="true"),
            DeclareLaunchArgument(
                "um982_protocol", default_value="uart", choices=["uart", "tcp"]
            ),
            DeclareLaunchArgument(
                "um982_port",
                default_value="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
            ),
            DeclareLaunchArgument("um982_baudrate", default_value="115200"),
            DeclareLaunchArgument("glim_odom_topic", default_value="/odom"),
            DeclareLaunchArgument(
                "um982_feedback_topic", default_value="/odometry/feedback"
            ),
            um982_driver,
            robot_state_publisher,
            um982_static_tf,
            local_ekf,
            # Let odom -> base_link initialize before the global filter needs it.
            TimerAction(period=1.0, actions=[global_ekf]),
        ]
    )
