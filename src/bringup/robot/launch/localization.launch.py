import os
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, PythonExpression
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_share = FindPackageShare("robot")
    robot_share_path = get_package_share_directory("robot")
    home_file = Path(get_package_share_directory("mission_manager")) / "config" / "home.yaml"
    with home_file.open() as stream:
        home_datum = yaml.safe_load(stream)["home"]

    enable_diagnostics_arg = DeclareLaunchArgument(
        "enable_diagnostics",
        default_value="true",
        # description="Launch generic topic heartbeat diagnostics for localization topics",
    )

    enable_glim_arg = DeclareLaunchArgument(
        "enable_glim",
        default_value="false",
        description="Launch GLIM here (leave false in the 2-machine split; Jetson hosts GLIM)",
    )
    enable_local_ekf_arg = DeclareLaunchArgument(
        "enable_local_ekf",
        default_value="false",
        description="Launch the Livox-IMU local EKF (off by default)",
    )
    enable_global_ekf_arg = DeclareLaunchArgument(
        "enable_global_ekf",
        default_value="false",
    )
    enable_navsat_transform_arg = DeclareLaunchArgument(
        "enable_navsat_transform",
        default_value="true",
    )
    enable_spatial_navsat_arg = DeclareLaunchArgument(
        "enable_spatial_navsat",
        default_value="false",
        description="Advanced Navigation Spatial navsat transform (adnav_driver submodule is not checked out)",
    )
    use_glim_fb_arg = DeclareLaunchArgument(
        "use_glim_fb",
        default_value="false",
        description="Fuse the Jetson GLIM /odom topic into the global EKF",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use the /clock topic (set true when replaying a rosbag).",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    global_ekf_config = PythonExpression(
        [
            "'ekf_global_glim.yaml' if '",
            LaunchConfiguration("use_glim_fb"),
            "'.lower() in ('true', '1', 'yes', 'on') else 'ekf_global.yaml'",
        ]
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
                "robot_description": ParameterValue(
                    robot_description,
                    value_type=str,
                )
            }
        ],
    )

    # CPUリソースが枯渇していた場合、pointcloud2に変換するのがボトルネックになるので、
    # 取れた点群を直に転送できるように、LiDAR Driver側でComposableNode起動？してください
    glim_node = Node(
        package="glim_ros",
        executable="glim_ros_node",
        name="glim_ros_node",
        output="screen",
        parameters=[
            {
                "config_path": PathJoinSubstitution(
                    [FindPackageShare("robot"), "config", "glim_config"]
                ),
                "use_sim_time": use_sim_time,
            }
        ],
        remappings=[("/glim_ros_node/odom", "/odom")],
        condition=IfCondition(LaunchConfiguration("enable_glim")),
    )

    spatial_navsat_transform_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="spatial_navsat_transform_node",
        output="screen",
        parameters=[
            {
                "world_frame": "odom",
                "use_sim_time": use_sim_time,
                "frequency": 10.0,
                "magnetic_declination_radians": 0.0,
                "yaw_offset": 0.0,
                "zero_altitude": True,
                "broadcast_utm_transform": False,
                "publish_filtered_gps": False,
                "use_odometry_yaw": False,
                "wait_for_datum": False,
            }
        ],
        remappings=[
            ("imu", "/adnav_driver/imu"),
            ("gps/fix", "/adnav_driver/nav_sat_fix"),
            ("odometry/filtered", "odometry/filtered/local"),
            ("odometry/gps", "/odometry/gps/spatial"),
        ],
        condition=IfCondition(LaunchConfiguration("enable_spatial_navsat")),
    )

    # The UM982 driver is launched separately because its transport settings
    # are deployment-specific.
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

    local_ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_local",
        output="screen",
        parameters=[
            PathJoinSubstitution([robot_share, "config", "ekf_local.yaml"]),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[("odometry/filtered", "odometry/filtered/local")],
        condition=IfCondition(LaunchConfiguration("enable_local_ekf")),
    )

    global_ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_global",
        output="screen",
        parameters=[
            PathJoinSubstitution([robot_share, "config", global_ekf_config]),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[("odometry/filtered", "odometry/filtered/global")],
        condition=IfCondition(LaunchConfiguration("enable_global_ekf")),
    )

    # Only finite observations are forwarded to the global filter. Its
    # GNSS-independent continuous observation comes from guarded Livox IMU.
    localization_input_guard = Node(
        package="robot",
        executable="localization_input_guard",
        name="localization_input_guard",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("enable_global_ekf")),
    )

    navsat_transform_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform_node",
        output="screen",
        # Keep the noisy datum/initialization messages local to this node.
        # Other nodes keep the launch-wide default log level.
        arguments=["--ros-args", "--log-level", "ERROR"],
        parameters=[
            {
                "world_frame": "odom",
                "use_sim_time": use_sim_time,
                "frequency": 10.0,
                "magnetic_declination_radians": 0.0,
                "yaw_offset": 0.0,
                "zero_altitude": True,
                "broadcast_utm_transform": True,
                "publish_filtered_gps": True,
                "use_odometry_yaw": True,
                "wait_for_datum": True,
                "datum": [
                    home_datum["latitude"], home_datum["longitude"], home_datum["yaw"]
                ],
            }
        ],
        remappings=[
            ("gps/fix", "/sensor/vehicle_gnss/fix/raw"),
            # Keep the first UM982 fix as the local odom origin.  navsat is
            # retained for /fromLL, but does not depend on the global EKF.
            ("odometry/filtered", "odometry/filtered/local"),
            ("odometry/gps", "/odometry/gps/um982"),
        ],
        condition=IfCondition(LaunchConfiguration("enable_navsat_transform")),
    )

    diagnostics_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_share_path, "launch", "diagnostics.launch.py")
        ),
        launch_arguments={
            "profile": "localization",
            "enable_global_ekf": LaunchConfiguration("enable_global_ekf"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("enable_diagnostics")),
    )

    return LaunchDescription(
        [
            enable_diagnostics_arg,
            enable_glim_arg,
            enable_local_ekf_arg,
            enable_global_ekf_arg,
            enable_navsat_transform_arg,
            enable_spatial_navsat_arg,
            use_glim_fb_arg,
            use_sim_time_arg,
            robot_state_publisher,
            glim_node,
            spatial_navsat_transform_node,
            um982_static_tf,
            local_ekf_node,
            global_ekf_node,
            localization_input_guard,
            navsat_transform_node,
            diagnostics_launch,
        ]
    )
