"""Nav2 nodes used by Task1.

The generic Nav2 navigation launch always starts docking and route servers.
Task1 needs collision monitoring for its command path but has no docking task,
so it owns the lifecycle list explicitly.
"""

import tempfile

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import Node


def _configure_bt_xmls(context):
    """Expand the BT frequency placeholder before Nav2 loads the XML files."""
    params_file = LaunchConfiguration("params_file").perform(context)
    with open(params_file, "r") as params_stream:
        params = yaml.safe_load(params_stream) or {}
    bt_params = params.get("bt_navigator", {}).get("ros__parameters", {})
    replanning_frequency = float(bt_params.get("replanning_frequency", 1.0))

    configured = {}
    for argument_name, config_name in (
        ("nav_to_pose_bt_xml", "configured_nav_to_pose_bt_xml"),
        ("nav_through_poses_bt_xml", "configured_nav_through_poses_bt_xml"),
    ):
        source_file = LaunchConfiguration(argument_name).perform(context)
        with open(source_file, "r") as xml_stream:
            xml = xml_stream.read()
        xml = xml.replace("__REPLANNING_FREQUENCY__", f"{replanning_frequency:.6g}")
        xml = xml.replace("<root ", '<root BTCPP_format="4" ', 1)
        with tempfile.NamedTemporaryFile(
            mode="w", prefix="task1_nav2_bt_", suffix=".xml", delete=False
        ) as configured_file:
            configured_file.write(xml)
            configured[config_name] = configured_file.name

    return [
        SetLaunchConfiguration(name, path) for name, path in configured.items()
    ]


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params = [
        RewrittenYaml(
            source_file=params_file,
            root_key=None,
            param_rewrites={
                "bt_navigator.ros__parameters.default_nav_to_pose_bt_xml":
                    LaunchConfiguration("configured_nav_to_pose_bt_xml"),
                "bt_navigator.ros__parameters.default_nav_through_poses_bt_xml":
                    LaunchConfiguration("configured_nav_through_poses_bt_xml"),
            },
            convert_types=True,
        ),
        {"use_sim_time": use_sim_time},
    ]
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "velocity_smoother",
        "collision_monitor",
        "bt_navigator",
        "waypoint_follower",
    ]

    return LaunchDescription([
        DeclareLaunchArgument("params_file"),
        DeclareLaunchArgument("nav_to_pose_bt_xml"),
        DeclareLaunchArgument("nav_through_poses_bt_xml"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        OpaqueFunction(function=_configure_bt_xmls),
        Node(package="nav2_controller", executable="controller_server",
             name="controller_server", output="screen", parameters=params,
             remappings=remappings + [("cmd_vel", "/cmd_vel_nav_raw")]),
        Node(package="nav2_smoother", executable="smoother_server",
             name="smoother_server", output="screen", parameters=params,
             remappings=remappings),
        Node(package="nav2_planner", executable="planner_server",
             name="planner_server", output="screen", parameters=params,
             remappings=remappings),
        Node(package="nav2_behaviors", executable="behavior_server",
             name="behavior_server", output="screen", parameters=params,
             remappings=remappings + [("cmd_vel", "/cmd_vel_nav_raw")]),
        Node(package="nav2_velocity_smoother", executable="velocity_smoother",
             name="velocity_smoother", output="screen", parameters=params,
             remappings=remappings + [
                 ("cmd_vel", "/cmd_vel_nav_raw"),
                 ("cmd_vel_smoothed", "/cmd_vel_smoothed"),
             ]),
        Node(package="nav2_collision_monitor", executable="collision_monitor",
             name="collision_monitor", output="screen", parameters=params,
             remappings=remappings),
        Node(package="nav2_bt_navigator", executable="bt_navigator",
             name="bt_navigator", output="screen", parameters=params,
             remappings=remappings),
        Node(package="nav2_waypoint_follower", executable="waypoint_follower",
             name="waypoint_follower", output="screen", parameters=params,
             remappings=remappings),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[{
                 "autostart": autostart,
                 "node_names": lifecycle_nodes,
                 "use_sim_time": use_sim_time,
             }]),
    ])
