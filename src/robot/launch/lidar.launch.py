# Livox LiDAR 起動。config の canonical は robot/config/livox/。
# vendored の config/launch は bringup では未使用。
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

# Livox ROS driver parameters, transcribed from
# livox_ros_driver2/launch_ROS2/msg_MID360_launch.py.
XFER_FORMAT = 0  # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
MULTI_TOPIC = 0  # 0-All LiDARs share the same topic, 1-One LiDAR one topic
DATA_SRC = 0  # 0-lidar, others-Invalid data src
PUBLISH_FREQ = 10.0  # frequency of publish, 5.0, 10.0, 20.0, 50.0, etc.
OUTPUT_DATA_TYPE = 0
FRAME_ID = "livox_frame"  # glim/TF が依存。必須。
LVX_FILE_PATH = "/home/livox/livox_test.lvx"
CMDLINE_INPUT_BD_CODE = "livox0000000001"

LIDAR_MODEL_TO_CONFIG_FILE = {
    "mid360": "MID360_config.json",
    "mid360s": "MID360S_jetson_config.json",
}


def launch_setup(context, *args, **kwargs):
    lidar_model = LaunchConfiguration("lidar_model").perform(context)
    config_file = LIDAR_MODEL_TO_CONFIG_FILE[lidar_model]
    enable_buoy_detection = (
        LaunchConfiguration("enable_buoy_detection").perform(context).lower()
        in ("true", "1", "yes", "on")
    )
    enable_glim = (
        LaunchConfiguration("enable_glim").perform(context).lower()
        in ("true", "1", "yes", "on")
    )
    glim_headless = (
        LaunchConfiguration("glim_headless").perform(context).lower()
        in ("true", "1", "yes", "on")
    )
    glim_backend = LaunchConfiguration("glim_backend").perform(context)
    glim_config_dir = (
        "glim_config_cpu" if glim_backend == "cpu"
        else "glim_config_headless" if glim_headless
        else "glim_config"
    )

    user_config_path = PathJoinSubstitution(
        [FindPackageShare("robot"), "config", "livox", config_file]
    )

    lidar_source = LaunchConfiguration("lidar_source").perform(context).lower()

    if lidar_source == "bag":
        # Replay a recorded mcap bag from INSIDE this container in place of the
        # real Livox driver. Publishing /livox/lidar + /livox/imu here (unique_ptr,
        # SensorDataQoS) keeps the zero-copy intra-process path to glim/pcl_det
        # identical to the live driver, only the data source changes. Used to
        # bench the intra-process pipeline on-device without the MID360 hardware.
        bag_path = LaunchConfiguration("bag_path").perform(context)
        bag_rate = float(LaunchConfiguration("bag_rate").perform(context))
        # restamp must stay OFF: Livox PointCloud2 carries per-point absolute
        # timestamps that GLIM uses, and rewriting only header.stamp desyncs
        # LiDAR (bag time) from IMU (rewritten), which crashes GLIM's estimator.
        # Keeping the bag timeline keeps points/IMU consistent; real-time pacing
        # is preserved by the replayer regardless.
        bag_restamp = (
            LaunchConfiguration("bag_restamp").perform(context).lower()
            in ("true", "1", "yes", "on")
        )
        # loop=false for GLIM benches: on loop the bag timestamps jump backward
        # by the bag duration, which GLIM reads as a time rewind and crashes its
        # estimator (IndeterminantLinearSystem). The bag (~586s) is far longer
        # than a bench window, so a single pass is enough.
        bag_loop = (
            LaunchConfiguration("bag_loop").perform(context).lower()
            in ("true", "1", "yes", "on")
        )
        source_component = ComposableNode(
            package="livox_bag_replayer",
            plugin="livox_bag_replayer::BagReplayerNode",
            name="livox_lidar_publisher",
            parameters=[
                {
                    "bag_path": bag_path,
                    "storage_id": "mcap",
                    "lidar_topic": "/livox/lidar",
                    "imu_topic": "/livox/imu",
                    "loop": bag_loop,
                    "rate": bag_rate,
                    "restamp": bag_restamp,
                    "frame_id": FRAME_ID,
                }
            ],
            remappings=[("/livox/imu", "/livox/imu_raw")],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
    else:
        source_component = ComposableNode(
            package="livox_ros_driver2",
            plugin="livox_ros::DriverNode",
            name="livox_lidar_publisher",
            parameters=[
                {
                    "xfer_format": XFER_FORMAT,
                    "multi_topic": MULTI_TOPIC,
                    "data_src": DATA_SRC,
                    "publish_freq": PUBLISH_FREQ,
                    "output_data_type": OUTPUT_DATA_TYPE,
                    "frame_id": FRAME_ID,
                    "lvx_file_path": LVX_FILE_PATH,
                    "user_config_path": user_config_path,
                    "cmdline_input_bd_code": CMDLINE_INPUT_BD_CODE,
                }
            ],
            remappings=[("/livox/imu", "/livox/imu_raw")],
            extra_arguments=[{"use_intra_process_comms": True}],
        )

    # Livox packets express acceleration in g. Keep the raw stream private to
    # this Jetson container and expose a sensor_msgs-compliant m/s^2 stream on
    # the historical /livox/imu topic for GLIM, the miniPC EKF, and logging.
    imu_scaler_component = ComposableNode(
        package="livox_ros_driver2",
        plugin="livox_ros::LivoxImuScaler",
        name="livox_imu_scaler",
        parameters=[
            {
                "input_topic": "/livox/imu_raw",
                "output_topic": "/livox/imu",
                "acceleration_scale": 9.80665,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    components = [source_component, imu_scaler_component]

    if enable_buoy_detection:
        components.append(
            ComposableNode(
                package="pcl_det",
                plugin="pcl_det::PclBuoyDetectionNode",
                name="pcl_bouy_det_node",
                parameters=[
                    {
                        "input_topic": "/livox/lidar",
                        "roi_topic": LaunchConfiguration("roi_topic"),
                        "output_topic": LaunchConfiguration("output_topic"),
                        "frame_id": LaunchConfiguration("detection_frame_id"),
                    }
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        )

    # GLIM is a registered rclcpp component (glim::GlimROS).  Keeping it in
    # the Livox container lets its PointCloud2 subscription use ROS 2
    # intra-process transport instead of serializing the cloud through DDS.
    # The SI-unit IMU scaler is in this same container, so GLIM can also receive
    # /livox/imu through the intra-process path.
    if enable_glim:
        components.append(
            ComposableNode(
                package="glim_ros",
                plugin="glim::GlimROS",
                name="glim_node",
                parameters=[
                    {
                        "config_path": PathJoinSubstitution(
                            [FindPackageShare("robot"), "config", glim_config_dir]
                        ),
                        "use_sim_time": False,
                    }
                ],
                remappings=[
                    ("/glim_node/odom", "/odom"),
                    # The miniPC EKFs are the only dynamic TF authorities.
                    # GLIM still publishes odometry, but its map/odom/base TF
                    # stays private to the Jetson and is not bridged.
                    ("/tf", "/glim/tf_unused"),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        )

    sensor_container = ComposableNodeContainer(
        package="rclcpp_components",
        executable="component_container_mt",
        name="livox_perception_container",
        namespace="/",
        output="screen",
        composable_node_descriptions=components,
    )

    return [sensor_container]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "lidar_model",
                default_value="mid360s",
                choices=["mid360", "mid360s"],
            ),
            DeclareLaunchArgument(
                "lidar_source",
                default_value="driver",
                choices=["driver", "bag"],
                description="driver=real Livox MID360, bag=in-container mcap "
                "replayer (intra-process bench)",
            ),
            DeclareLaunchArgument(
                "bag_path",
                default_value="/home/ibo/njord2026_ws/rosbag2_2026_07_21-16_22_30",
                description="mcap bag replayed when lidar_source:=bag",
            ),
            DeclareLaunchArgument(
                "bag_rate",
                default_value="1.0",
                description="Playback speed multiplier for lidar_source:=bag",
            ),
            DeclareLaunchArgument(
                "bag_loop",
                default_value="true",
                description="Loop the bag. Set false for GLIM benches: looping "
                "rewinds bag timestamps and crashes GLIM's estimator",
            ),
            DeclareLaunchArgument(
                "bag_restamp",
                default_value="false",
                description="Rewrite header.stamp to now(); keep false for GLIM "
                "(Livox per-point timestamps would desync from IMU otherwise)",
            ),
            DeclareLaunchArgument("enable_buoy_detection", default_value="false"),
            DeclareLaunchArgument(
                "enable_glim",
                default_value="true",
                description="Load GLIM into the Livox component container",
            ),
            DeclareLaunchArgument(
                "glim_headless",
                default_value="false",
                description="Use glim_config_headless (no GUI viewer extensions) "
                "instead of glim_config; required when no X11/OpenGL display "
                "is available",
            ),
            DeclareLaunchArgument(
                "glim_backend",
                default_value="gpu",
                choices=["gpu", "cpu"],
                description="Select the GLIM config directory "
                "(glim_config for gpu, glim_config_cpu for cpu)",
            ),
            DeclareLaunchArgument("roi_topic", default_value="/buoy_roi"),
            DeclareLaunchArgument("output_topic", default_value="/buoy_detections"),
            DeclareLaunchArgument("detection_frame_id", default_value="base_link"),
            OpaqueFunction(function=launch_setup),
        ]
    )
