"""One isolated hardware-free mission; starts MANUAL/IDLE."""

import tempfile
from pathlib import Path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory as share
from njord_sim.configuration import configurations
from njord_sim.scenario import scenario, TASKS


def launch(context):
    task = LaunchConfiguration("task").perform(context)
    model = LaunchConfiguration("model").perform(context)
    seed = int(LaunchConfiguration("seed").perform(context))
    noise = LaunchConfiguration("noise").perform(context) == "true"
    disturbance = LaunchConfiguration("disturbance").perform(context) == "true"
    directory = LaunchConfiguration("config_dir").perform(context) or tempfile.mkdtemp(
        prefix="njord_sim_"
    )
    nav, policy, registry = configurations(task, directory)
    s = scenario(task)

    def include(package, file, args={}):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(share(package)) / "launch" / file)),
            launch_arguments=args.items(),
        )

    nodes = [
        Node(
            package="njord_sim",
            executable="environment",
            parameters=[
                {
                    "task": task,
                    "model": model,
                    "noise": noise,
                    "seed": seed,
                    "disturbance": disturbance,
                    "encounter": LaunchConfiguration("encounter"),
                    "dropout_after": float(
                        LaunchConfiguration("dropout_after").perform(context)
                    ),
                }
            ],
            output="screen",
        ),
        include("control_manager", "control.launch.py", {"policy_file": policy}),
        Node(
            package="mission_manager",
            executable="mission_manager_node",
            parameters=[
                {"active_nav2_profile": s.task.nav2_profile, "registry_file": registry}
            ],
            output="screen",
        ),
        include(
            "task1_sim",
            "task1_navigation.launch.py",
            {
                "params_file": nav,
                "nav_to_pose_bt_xml": str(
                    Path(share("robot"))
                    / "config/navigate_to_pose_w_replanning_and_recovery.xml"
                ),
                "nav_through_poses_bt_xml": str(
                    Path(share("task1_sim"))
                    / "behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml"
                ),
            },
        ),
        include("alert_lamp", "alert_lamp.launch.py"),
    ]
    if task.startswith("task1") or task == "task4":
        nodes.append(
            Node(
                package="buoy_obstacle_publisher",
                executable="cardinal_wall_publisher",
                parameters=[
                    {
                        "map_frame": "odom",
                        "course_bounds": [-150.0, 150.0, -150.0, 150.0],
                        "max_wall_length_m": 13.0,
                        "wall_width_m": 0.2,
                        "point_spacing_m": 0.1,
                        "confirmations_required": 2,
                        "true_north_heading_topic": "/sensor/vehicle_gnss/compass/raw",
                        "true_north_confirmations_required": 3,
                        "wall_enable_topic": "/task4/buoy_wall_enable"
                        if task == "task4"
                        else "/task1/cardinal_wall_enable",
                        "retirement_heading_topic": "/task4/gps14_to_gps16_heading"
                        if task == "task4"
                        else "/task1/gps3_to_gps4_heading",
                        "retire_passed_cardinal_walls_from_base_pose": True,
                        "retirement_margin_m": 1.0,
                        "retirement_confirmations_required": 5,
                    }
                ],
                output="screen",
            )
        )
    if LaunchConfiguration("foxglove").perform(context) == "true":
        nodes.append(
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                parameters=[
                    {
                        "port": int(
                            LaunchConfiguration("foxglove_port").perform(context)
                        ),
                        "address": "127.0.0.1",
                        "capabilities": ["services", "parameters", "connectionGraph"],
                    }
                ],
                output="screen",
            )
        )
    if task == "task2":
        from njord_sim.task2 import nodes as task2_nodes

        nodes.extend(task2_nodes(directory, seed))
    if model == "mmg":
        description = (
            Path(share("robot")) / "urdf/robot.urdf_modified.urdf"
        ).read_text()
        nodes.extend(
            [
                Node(
                    package="dutyed_tf_pub_with_disturbance",
                    executable="dutyed_tf_pub_with_disturbance_node",
                    parameters=[
                        str(
                            Path(share("dutyed_tf_pub_with_disturbance"))
                            / "config/node_config.yaml"
                        ),
                        {
                            "publish_tf": False,
                            "topic_odom": "/sim/physics/odom",
                            "initial_pose_xyyaw": list(s.initial),
                            "thruster_force_sign": [1.0, 1.0, 1.0, 1.0],
                            "disturbance.seed": seed,
                            "disturbance.sigma_u": 0.12 if disturbance else 0.0,
                            "disturbance.sigma_v": 0.12 if disturbance else 0.0,
                            "disturbance.sigma_r": 0.04 if disturbance else 0.0,
                        },
                    ],
                    output="screen",
                ),
                Node(
                    package="thruster_driver",
                    executable="thruster_driver_node",
                    parameters=[
                        str(Path(share("thruster_driver")) / "config/config.yaml"),
                        {
                            "robot_description": description,
                            "control.dob.enable": False,
                            "allocation.wrench_sign": [1.0, 1.0, 1.0],
                            "thrusters.reverse": [False] * 4,
                        },
                    ],
                    output="screen",
                ),
            ]
        )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("task", default_value="task1", choices=list(TASKS)),
            DeclareLaunchArgument(
                "model", default_value="kinematic", choices=["kinematic", "mmg"]
            ),
            DeclareLaunchArgument(
                "noise", default_value="false", choices=["false", "true"]
            ),
            DeclareLaunchArgument("seed", default_value="2026"),
            DeclareLaunchArgument("disturbance", default_value="false"),
            DeclareLaunchArgument("encounter", default_value="head_on"),
            DeclareLaunchArgument("config_dir", default_value=""),
            DeclareLaunchArgument(
                "foxglove", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument("foxglove_port", default_value="8767"),
            DeclareLaunchArgument("dropout_after", default_value="0.0"),
            OpaqueFunction(function=launch),
        ]
    )
