"""Role-specific Zenoh and critical-link networking for real hardware."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def _role_condition(role):
    return IfCondition(
        PythonExpression(["'", LaunchConfiguration("role"), "' == '", role, "'"])
    )


def generate_launch_description():
    role = LaunchConfiguration("role")
    enable_zenoh_bridge = LaunchConfiguration("enable_zenoh_bridge")
    enable_critical_link = LaunchConfiguration("enable_critical_link")

    bridge_config = PathJoinSubstitution(
        [
            FindPackageShare("robot"),
            "config",
            "zenoh",
            PythonExpression(["'bridge_", role, ".json5'"]),
        ]
    )

    zenoh_bridge = ExecuteProcess(
        # ROS_DOMAIN_ID in the parent bringup belongs to its local ROS nodes.
        # The bridge must instead use the domain in its role-specific JSON5
        # configuration, so explicitly clear that inherited variable.
        cmd=["env", "-u", "ROS_DOMAIN_ID", "zenoh-bridge-ros2dds", "-c", bridge_config],
        output="screen",
        condition=IfCondition(enable_zenoh_bridge),
    )

    ground_sender = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("critical_link"), "launch", "ground_sender.launch.py"]
            )
        ),
        condition=_role_condition("groundpc"),
    )
    vessel_receiver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("critical_link"), "launch", "vessel_receiver.launch.py"]
            )
        ),
        condition=_role_condition("minipc"),
    )
    critical_link = GroupAction(
        scoped=True,
        condition=IfCondition(enable_critical_link),
        actions=[ground_sender, vessel_receiver],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "role", choices=["groundpc", "jetson", "minipc"],
                description="Machine role used to select the Zenoh bridge and critical-link node.",
            ),
            DeclareLaunchArgument("enable_zenoh_bridge", default_value="true"),
            DeclareLaunchArgument(
                "enable_critical_link",
                default_value="true",
                description="Start sender on groundpc and receiver on minipc; Jetson has no critical-link node.",
            ),
            zenoh_bridge,
            critical_link,
        ]
    )
