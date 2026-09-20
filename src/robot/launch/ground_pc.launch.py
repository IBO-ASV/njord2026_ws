import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def generate_launch_description():
    zed2i_share_path = get_package_share_directory("zed2i_driver")
    front_video_port = LaunchConfiguration("front_video_port")
    front_video_topic = LaunchConfiguration("front_video_topic")
    back_video_port = LaunchConfiguration("back_video_port")
    back_video_codec = LaunchConfiguration("back_video_codec")
    back_video_topic = LaunchConfiguration("back_video_topic")
    back_video_jitter_latency_ms = LaunchConfiguration(
        "back_video_jitter_latency_ms"
    )
    back_cam_jpeg_video_port = LaunchConfiguration("back_cam_jpeg_video_port")
    enable_ntrip_caster = LaunchConfiguration("enable_ntrip_caster")
    enable_foxglove_bridge = LaunchConfiguration("enable_foxglove_bridge")
    enable_livox_gui_telemetry = LaunchConfiguration("enable_livox_gui_telemetry")

    livox_splat_mapper = Node(
        package="livox_gui_telemetry", executable="livox_splat_mapper", name="livox_splat_mapper", output="screen",
        condition=IfCondition(enable_livox_gui_telemetry),
    )
    ntrip_caster_config = LaunchConfiguration("ntrip_caster_config")
    runtime_parameters_file = LaunchConfiguration("runtime_parameters_file")

    runtime_parameter_sync = Node(
        package="robot",
        executable="runtime_parameter_sync.py",
        name="runtime_parameter_sync",
        output="screen",
        parameters=[{
            "role": "groundpc",
            "config_file": runtime_parameters_file,
            "publish_updates": True,
        }],
    )

    ground_video_receiver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed2i_share_path, "launch", "ground_video_receiver.launch.py")
        ),
        launch_arguments={
            "port": front_video_port,
            "topic": front_video_topic,
        }.items(),
    )

    back_cam_jpeg_receiver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed2i_share_path, "launch", "ground_video_receiver.launch.py")
        ),
        launch_arguments={
            "port": back_cam_jpeg_video_port,
            "topic": "/ground_video/back_cam_jpeg/compressed",
        }.items(),
    )

    back_cam_h26x_receiver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed2i_share_path, "launch", "ground_h26x_receiver.launch.py")
        ),
        launch_arguments={
            "port": back_video_port,
            "codec": back_video_codec,
            "topic": back_video_topic,
            "jitter_latency_ms": back_video_jitter_latency_ms,
        }.items(),
    )

    foxglove_bridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("foxglove_bridge"), "launch", "foxglove_bridge_launch.xml"]
            )
        ),
        condition=IfCondition(enable_foxglove_bridge),
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        emulate_tty=True,
        # The canonical /joy is vessel-owned.  Feeding it here would let the
        # Zenoh bridge bypass critical_link and contend with the receiver.
        remappings=[("/joy", "/critical_link/input/joy")],
    )

    ground_station_heartbeat = Node(
        package="simple_manual",
        executable="ground_station_heartbeat_node",
        name="ground_station_heartbeat",
        output="screen",
        # As with joystick input, only critical_link_receiver may publish the
        # canonical vessel heartbeat.
        parameters=[
            {"topic": "/critical_link/input/heartbeat", "period_sec": 1.0}
        ],
    )

    actual_route = Node(
        package="tf_frame_arrow_publisher",
        executable="full_path_publisher",
        name="actual_route_publisher",
        output="screen",
        parameters=[
            {
                "marker_topic": "/actual_path_marker",
                "parent_frame": "odom",
                "child_frame": "base_link",
            }
        ],
    )

    planned_route = Node(
        package="tf_frame_arrow_publisher",
        executable="planned_path_marker_publisher",
        name="planned_route_publisher",
        output="screen",
        parameters=[{
            "path_topic": "/planned_path_pruned",
            "marker_topic": "/planned_path_marker",
        }],
    )

    ground_waypoints = Node(
        package="waypoint_publisher",
        executable="ground_waypoint_geo_publisher",
        name="ground_waypoint_geo_publisher",
        output="screen",
    )

    ntrip_caster = ExecuteProcess(
        cmd=[
            PathJoinSubstitution(
                [FindPackagePrefix("ntripcaster"), "lib", "ntripcaster", "ntripcaster"]
            ),
            ntrip_caster_config,
        ],
        output="screen",
        condition=IfCondition(enable_ntrip_caster),
    )

    networking_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("robot"), "launch", "networking.launch.py"]
            )
        ),
        launch_arguments={
            "role": "groundpc",
            "enable_zenoh_bridge": LaunchConfiguration("enable_zenoh_bridge"),
            "enable_critical_link": LaunchConfiguration("enable_critical_link"),
            "zenoh_transport": LaunchConfiguration("zenoh_transport"),
        }.items(),
    )

    return LaunchDescription(
        [
            # The two receivers ingest different RTP streams and must keep
            # separate UDP ports and ROS topics.
            DeclareLaunchArgument("front_video_port", default_value="5600"),
            DeclareLaunchArgument(
                "front_video_topic", default_value="/ground_video/image/compressed"
            ),
            DeclareLaunchArgument("back_video_port", default_value="5601"),
            DeclareLaunchArgument("back_cam_jpeg_video_port", default_value="5602"),
            DeclareLaunchArgument(
                "back_video_codec", default_value="h264", choices=["h264", "h265"]
            ),
            DeclareLaunchArgument(
                "back_video_topic", default_value="/ground_video/back_cam/image_raw"
            ),
            DeclareLaunchArgument(
                "back_video_jitter_latency_ms",
                default_value="50",
                description="RTP jitter buffer for the back-camera stream. "
                "Use 0 only for controlled low-latency tests.",
            ),
            DeclareLaunchArgument("enable_ntrip_caster", default_value="true"),
            DeclareLaunchArgument(
                "runtime_parameters_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robot"), "runtime_params", "runtime_parameters.yaml"]
                ),
                description="Editable runtime parameters distributed from the ground PC.",
            ),
            DeclareLaunchArgument(
                "enable_foxglove_bridge",
                default_value="false",
                description="Expose vessel telemetry and waypoint markers to the Ground PC Foxglove GUI.",
            ),
            DeclareLaunchArgument("enable_livox_gui_telemetry", default_value="true", description="Integrate bounded Livox GUI clouds into an odom map."),
            livox_splat_mapper,
            DeclareLaunchArgument(
                "enable_zenoh_bridge",
                default_value="true",
                description="Start the Ground PC zenoh-bridge-ros2dds process.",
            ),
            DeclareLaunchArgument(
                "zenoh_transport", default_value="tailscale", choices=["direct", "tailscale"],
                description="Use Tailscale by default; pass direct to use the existing 10.42.0.0/24 link.",
            ),
            DeclareLaunchArgument(
                "enable_critical_link",
                default_value="true",
                description="Start critical_link_sender for joystick and Ground heartbeat.",
            ),
            DeclareLaunchArgument(
                "ntrip_caster_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ntripcaster"), "config", "ntripcaster.json"]
                ),
                description="Caster config: 0.0.0.0:2101, SOURCE osw/RTCM3",
            ),
            joy_node,
            ground_station_heartbeat,
            actual_route,
            planned_route,
            ground_waypoints,
            runtime_parameter_sync,
            ground_video_receiver_launch,
            back_cam_h26x_receiver_launch,
            back_cam_jpeg_receiver_launch,
            foxglove_bridge_launch,
            ntrip_caster,
            networking_launch,
        ]
    )
