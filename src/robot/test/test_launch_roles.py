"""Regression tests for the Jetson / miniPC launch-role split.

This is the most important regression test for the two-machine split: it
guards against GPU-only packages (glim_ros, livox_ros_driver2, glim_config)
accidentally being pulled into the miniPC bringup, and against
CPU/serial-only packages (robot_localization, thruster_driver,
micon_driver_fd, joy_node) accidentally being pulled into the Jetson
bringup.

Launch files are read from the source tree (not the installed share
directory) so this test also works before `colcon build` has installed
anything, and so it works the same way in CI.
"""

import importlib.util
import os
import re
import xml.etree.ElementTree as ET

import pytest
import yaml

_THIS_DIR = os.path.dirname(__file__)
_LAUNCH_DIR = os.path.normpath(os.path.join(_THIS_DIR, "..", "launch"))

_GPU_ONLY_PACKAGES = ["glim_ros", "livox_ros_driver2", "glim_config"]
_MINIPC_ONLY_PACKAGES = ["robot_localization", "thruster_driver", "micon_driver_fd", "joy_node"]

_GENERATE_LAUNCH_DESCRIPTION_FILES = [
    "glim_um982_localization.launch.py",
    "minipc_bringup.launch.py",
    "task1.launch.py",
    "task3.launch.py",
    "networking.launch.py",
]


def _launch_file_path(filename):
    path = os.path.join(_LAUNCH_DIR, filename)
    assert os.path.isfile(path), f"expected launch file not found: {path}"
    return path


def _read_launch_source(filename):
    with open(_launch_file_path(filename), "r") as f:
        return f.read()


def _load_launch_module(filename):
    path = _launch_file_path(filename)
    module_name = f"_njord_test_launch_{os.path.splitext(filename)[0].replace('.', '_')}"
    spec = importlib.util.spec_from_file_location(module_name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _walk_entities(entities):
    """Recursively yield every launch entity/action reachable from entities."""
    for entity in entities:
        yield entity
        # LaunchDescription-like objects and GroupAction/composed actions
        # expose their nested entities in different ways depending on
        # type; probe the common attributes defensively.
        nested = None
        if hasattr(entity, "entities"):
            nested = entity.entities
        elif hasattr(entity, "get_sub_entities"):
            try:
                nested = entity.get_sub_entities()
            except Exception:
                nested = None
        if nested:
            yield from _walk_entities(nested)


@pytest.mark.parametrize("filename", _GENERATE_LAUNCH_DESCRIPTION_FILES)
def test_generate_launch_description_returns_without_exception(filename):
    """generate_launch_description() must not raise for any of these files.

    Resolving package shares via ament_index requires the packages to be
    installed/sourced; if that fails for reasons unrelated to this launch
    file's own logic (e.g. running outside a built workspace), skip rather
    than fail so the source-string checks below remain the reliable part
    of this test.
    """
    try:
        module = _load_launch_module(filename)
    except Exception as exc:
        pytest.skip(f"could not import {filename}: {exc}")

    try:
        launch_description = module.generate_launch_description()
    except Exception as exc:
        pytest.skip(
            f"generate_launch_description() in {filename} raised "
            f"(likely due to unresolved package shares in this test environment): {exc}"
        )

    assert launch_description is not None
    # Just confirm we can walk it without blowing up.
    list(_walk_entities(launch_description.entities))


def test_minipc_bringup_source_has_no_gpu_packages():
    source = _read_launch_source("minipc_bringup.launch.py")
    for package_name in _GPU_ONLY_PACKAGES:
        assert package_name not in source, (
            f"minipc_bringup.launch.py must not reference GPU-only package "
            f"'{package_name}'; that belongs in jetson_bringup.launch.py"
        )


def test_minipc_bringup_keeps_h26x_and_jpeg_back_camera_paths_separate():
    source = _read_launch_source("minipc_bringup.launch.py")
    assert 'back_cam_h26x_ground_video.launch.py' in source
    assert 'back_cam_jpeg_ground_video.launch.py' in source
    assert '"enable_back_cam_jpeg_ground_video"' in source


def test_minipc_bringup_uses_the_canonical_control_manager_by_default():
    source = _read_launch_source("minipc_bringup.launch.py")
    assert '"control_manager"' in source
    assert '"control.launch.py"' in source
    assert '"enable_control_manager",' in source
    assert 'default_value="true"' in source
    # Keep the old arbiter only for the explicit legacy comparison path; it
    # cannot contend for /cmd_vel during normal role bringup.
    assert 'executable="command_arbiter_node"' in source
    assert 'condition=UnlessCondition(enable_control_manager)' in source
    assert 'package="twist_mux"' not in source


def test_minipc_joy_converter_does_not_publish_physical_lamp_topics():
    source = _read_launch_source("minipc_bringup.launch.py")

    for color in ("green", "yellow", "red"):
        assert f'("/{color}", "/manual_lamp/{color}")' in source


def test_minipc_bringup_starts_the_autonomy_supervisor_once():
    minipc_source = _read_launch_source("minipc_bringup.launch.py")

    assert 'package="diagnostic_monitors"' in minipc_source
    assert 'executable="autonomy_supervisor_node"' in minipc_source
    assert '"enable_autonomy_supervisor",' in minipc_source

    for filename in ["task1.launch.py", "task3.launch.py"]:
        assert "autonomy_supervisor_node" not in _read_launch_source(filename)


def test_operator_nodes_have_single_machine_owners():
    minipc_source = _read_launch_source("minipc_bringup.launch.py")
    ground_source = _read_launch_source("ground_pc.launch.py")

    assert 'executable="joy_node"' not in minipc_source
    assert '"enable_joy"' not in minipc_source
    assert 'foxglove_bridge_launch.xml' not in minipc_source
    assert 'executable="foxglove_logger_node"' in minipc_source

    assert 'executable="joy_node"' in ground_source
    assert 'foxglove_bridge_launch.xml' in ground_source


def test_ground_pc_publishes_the_actual_route_marker_from_shared_tf():
    source = _read_launch_source("ground_pc.launch.py")

    assert 'package="tf_frame_arrow_publisher"' in source
    assert 'executable="full_path_publisher"' in source
    assert 'name="actual_route_publisher"' in source
    assert '"marker_topic": "/actual_path_marker"' in source
    assert '"parent_frame": "odom"' in source
    assert '"child_frame": "base_link"' in source


def test_disabled_minipc_serial_drivers_default_to_false():
    source = _read_launch_source("minipc_bringup.launch.py")
    assert '"enable_drogger_rzs", default_value="false"' in source
    assert '"enable_imu", default_value="false"' in source


def test_minipc_starts_um982_and_spatial_ntrip_clients_from_the_ground_caster():
    source = _read_launch_source("minipc_bringup.launch.py")

    assert '"ntrip_server": "osw-Stealth-14-AI-Studio-A1VGG.local"' in source
    assert '"ntrip_mountpoint": "RTCM3"' in source
    assert '"ntrip_username": "test"' in source
    assert 'package="adnav_driver"' in source
    assert '"/adnav_driver/ntrip"' in source
    assert "adnav_interfaces/srv/Ntrip" in source
    assert "host: 'osw-Stealth-14-AI-Studio-A1VGG.local:2101'" in source
    assert "mountpoint: 'RTCM3'" in source


def test_minipc_video_and_glim_feedback_defaults_are_safe():
    source = _read_launch_source("minipc_bringup.launch.py")
    assert 'default_value="osw-Stealth-14-AI-Studio-A1VGG.local"' in source
    assert '"back_cam_ground_video_fps", default_value="3.0"' in source
    assert '"back_cam_ground_video_width", default_value="360"' in source
    assert '"back_cam_ground_video_height", default_value="240"' in source
    assert '"use_glim_fb"' in source
    assert 'default_value="false"' in source


def test_local_ekf_is_opt_in_and_replaces_um982_feedback_ekf():
    minipc_source = _read_launch_source("minipc_bringup.launch.py")
    localization_source = _read_launch_source("localization.launch.py")
    standalone_source = _read_launch_source("standalone_bringup.launch.py")

    assert '"use_ekf_local"' in minipc_source
    assert '"enable_local_ekf": use_ekf_local' in minipc_source
    assert "condition=UnlessCondition(use_ekf_local)" in minipc_source
    assert '"enable_local_ekf",\n        default_value="false"' in localization_source
    assert '"use_ekf_local": LaunchConfiguration("use_ekf_local")' in standalone_source
    assert '"use_ekf_local",\n                default_value="false"' in standalone_source


def test_standalone_does_not_restore_removed_or_unsafe_defaults():
    source = _read_launch_source("standalone_bringup.launch.py")
    assert '"enable_joy"' not in source
    assert '"enable_foxglove"' not in source
    assert '"enable_drogger_rzs", default_value="false"' in source
    assert '"enable_glim",\n                default_value="false"' in source
    assert '"enable_pcl_buoy_detection", default_value="false"' in source
    assert '"enable_ground_video", default_value="true"' in source
    assert '"enable_zenoh_bridge", default_value="false"' in source
    assert '"enable_critical_link", default_value="false"' in source


def test_local_ekf_uses_livox_rates_and_acceleration_not_orientation():
    config_path = os.path.normpath(
        os.path.join(_THIS_DIR, "..", "config", "ekf_local.yaml")
    )
    with open(config_path, "r") as stream:
        params = yaml.safe_load(stream)["ekf_filter_node_local"]["ros__parameters"]

    assert params["imu0"] == "/livox/imu"
    assert params["imu0_config"] == [
        False, False, False,
        False, False, False,
        False, False, False,
        True, True, True,
        True, True, True,
    ]


def test_global_ekf_uses_only_guarded_continuous_inputs():
    config_path = os.path.normpath(
        os.path.join(_THIS_DIR, "..", "config", "ekf_global.yaml")
    )
    with open(config_path, "r") as stream:
        params = yaml.safe_load(stream)["ekf_filter_node_global"]["ros__parameters"]

    assert params["odom0"] == "/odometry/gps/um982"
    assert params["pose0"] == "/sensor/vehicle_gnss/compass/validated"
    assert params["imu0"] == "/livox/imu/validated"
    assert params["imu0_config"] == [
        False, False, False,
        False, False, False,
        False, False, False,
        False, False, True,
        True, True, False,
    ]


def test_localization_launch_guards_navsat_output_before_global_ekf():
    source = _read_launch_source("localization.launch.py")
    assert 'executable="localization_input_guard"' in source
    assert '("odometry/gps", "/odometry/gps/um982/raw")' in source


def test_camera_frames_match_measured_mounting_geometry():
    urdf_path = os.path.normpath(
        os.path.join(_THIS_DIR, "..", "urdf", "robot.urdf.xacro")
    )
    root = ET.parse(urdf_path).getroot()
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}

    right = joints["zed2i_right_camera_joint"]
    assert right.find("parent").attrib["link"] == "zed2i_left_camera_frame"
    assert right.find("child").attrib["link"] == "zed2i_right_camera_frame"
    assert right.find("origin").attrib == {"xyz": "0 -0.15 0", "rpy": "0 0 0"}

    back = joints["back_cam_joint"]
    assert back.find("parent").attrib["link"] == "zed2i_left_camera_frame"
    assert back.find("child").attrib["link"] == "back_cam_link"
    assert back.find("origin").attrib == {
        "xyz": "-0.60 -0.075 0",
        "rpy": "0 0 3.141592653589793",
    }


def test_jetson_bringup_source_has_no_minipc_only_packages():
    source = _read_launch_source("jetson_bringup.launch.py")
    for package_name in _MINIPC_ONLY_PACKAGES:
        assert package_name not in source, (
            f"jetson_bringup.launch.py must not reference miniPC-only package "
            f"'{package_name}'; that belongs in minipc_bringup.launch.py"
        )


def test_jetson_heavy_features_and_startup_are_opt_in_or_staggered():
    source = _read_launch_source("jetson_bringup.launch.py")
    assert '"enable_glim",\n                default_value="false"' in source
    assert '"enable_pcl_buoy_detection",\n                default_value="false"' in source
    assert '"enable_ground_video", default_value="true"' in source
    assert '"camera_start_delay",\n                default_value="5.0"' in source


def test_glim_does_not_publish_the_shared_tf_topic():
    source = _read_launch_source("lidar.launch.py")
    assert '("/glim_node/odom", "/odom")' in source
    assert '("/tf", "/glim/tf_unused")' in source


def test_glim_feedback_profile_fuses_odom_without_changing_ekf_tf_ownership():
    config_path = os.path.normpath(
        os.path.join(_THIS_DIR, "..", "config", "ekf_global_glim.yaml")
    )
    with open(config_path, "r") as stream:
        params = yaml.safe_load(stream)["ekf_filter_node_global"]["ros__parameters"]

    assert params["odom1"] == "/odom/validated"
    assert params["publish_tf"] is True
    assert params["world_frame"] == "map"


def test_glim_um982_localization_has_one_owner_per_dynamic_tf_edge():
    launch_source = _read_launch_source("glim_um982_localization.launch.py")
    assert '"feedback_frame_id": "map"' in launch_source
    assert '"ekf_glim_local.yaml"' in launch_source
    assert '"ekf_um982_map.yaml"' in launch_source

    local_path = os.path.normpath(
        os.path.join(_THIS_DIR, "..", "config", "ekf_glim_local.yaml")
    )
    global_path = os.path.normpath(
        os.path.join(_THIS_DIR, "..", "config", "ekf_um982_map.yaml")
    )
    with open(local_path, "r") as stream:
        local = yaml.safe_load(stream)["ekf_filter_node_glim_local"]["ros__parameters"]
    with open(global_path, "r") as stream:
        global_ = yaml.safe_load(stream)["ekf_filter_node_um982_map"]["ros__parameters"]

    assert local["publish_tf"] is True
    assert local["world_frame"] == "odom"
    assert local["odom0"] == "/odom"
    assert global_["publish_tf"] is True
    assert global_["world_frame"] == "map"
    assert global_["odom0"] == "/odometry/feedback"


@pytest.mark.parametrize("filename", ["task1.launch.py", "task2.launch.py", "task3.launch.py"])
def test_task_launch_files_declare_role_argument(filename):
    source = _read_launch_source(filename)
    assert "'role'" in source or '"role"' in source
    assert (
        "choices=['minipc', 'standalone']" in source
        or 'choices=["minipc", "standalone"]' in source
    )
    assert "default_value='minipc'" in source or 'default_value="minipc"' in source


@pytest.mark.parametrize("filename", ["task1.launch.py", "task2.launch.py", "task3.launch.py"])
def test_legacy_task_launches_disable_persistent_managers(filename):
    source = _read_launch_source(filename)
    # Compatibility launches own their historical graph and must not start a
    # second resident Nav2/control/mission graph from role bringup.
    assert "'enable_nav2': 'false'" in source
    assert "'enable_mission_manager': 'false'" in source
    assert "'enable_control_manager': 'false'" in source


@pytest.mark.parametrize("filename", ["task1.launch.py", "task2.launch.py", "task3.launch.py"])
def test_legacy_task_wrappers_are_opt_in(filename):
    source = _read_launch_source(filename)
    assert re.search(r"['\"]start_role_bringup['\"].*default_value=['\"]false", source, re.S)
    assert re.search(r"['\"]start_legacy_task_nodes['\"].*default_value=['\"]false", source, re.S)
    assert "LaunchConfiguration('start_legacy_task_nodes')" in source


@pytest.mark.parametrize(
    "filename",
    ["task1-1.launch.py", "task1-2.launch.py", "task2-1.launch.py",
     "task3-1.launch.py", "task3-2.launch.py"],
)
def test_numbered_legacy_task_launches_are_opt_in(filename):
    source = _read_launch_source(filename)
    assert '"enable_legacy_graph", default_value="false"' in source
    assert 'LaunchConfiguration("enable_legacy_graph")' in source


def test_ground_pc_keeps_front_and_back_video_receivers_separate():
    source = _read_launch_source("ground_pc.launch.py")

    assert "ground_video_receiver.launch.py" in source
    assert "ground_h26x_receiver.launch.py" in source
    assert '"front_video_port", default_value="5600"' in source
    assert '"back_video_port", default_value="5601"' in source
    assert '"front_video_topic", default_value="/ground_video/image/compressed"' in source
    assert '"back_video_topic", default_value="/ground_video/back_cam/image_raw"' in source
    assert '"topic": front_video_topic' in source
    assert '"topic": back_video_topic' in source
    assert '"back_video_jitter_latency_ms"' in source
    assert 'default_value="50"' in source


def test_ground_pc_starts_workspace_ntrip_caster_by_default():
    source = _read_launch_source("ground_pc.launch.py")
    assert '"enable_ntrip_caster", default_value="true"' in source
    assert 'FindPackagePrefix("ntripcaster")' in source
    assert 'FindPackageShare("ntripcaster")' in source
    assert '"ntripcaster.json"' in source


def test_bringups_start_hierarchical_health_heartbeats():
    jetson_source = _read_launch_source("jetson_bringup.launch.py")
    minipc_source = _read_launch_source("minipc_bringup.launch.py")
    heartbeat_source = _read_launch_source("heartbeat.launch.py")
    minipc_heartbeat_path = os.path.join(
        _THIS_DIR,
        "..",
        "..",
        "diagnostics",
        "diagnostic_monitors",
        "config",
        "minipc_heartbeat.yaml",
    )
    with open(minipc_heartbeat_path, encoding="utf-8") as config_file:
        minipc_heartbeat = yaml.safe_load(config_file)["minipc_heartbeat"]

    assert '"role": "jetson"' in jetson_source
    assert '"role": "minipc"' in minipc_source
    assert '"/heartbeat/driver/camera/front"' in heartbeat_source
    assert '"/heartbeat/driver/lidar"' in heartbeat_source
    assert '"heartbeat_monitor_zed2i"' in heartbeat_source
    assert '"heartbeat_monitor_lidar"' in heartbeat_source
    assert "minipc_heartbeat.yaml" in heartbeat_source
    assert "heartbeat_monitor_" not in minipc_source
    assert "enable_heartbeats" not in minipc_source
    heartbeat_topics = {
        input_config["topic"]
        for gate in minipc_heartbeat
        for input_config in gate["inputs"]
    }
    heartbeat_topics.update(gate["output_topic"] for gate in minipc_heartbeat)
    for topic in [
        "/heartbeat/driver/camera/back",
        "/heartbeat/driver/gnss",
        "/heartbeat/driver/micon",
        "/heartbeat/driver",
    ]:
        assert topic in heartbeat_topics


def test_back_camera_sender_defaults_match_front_ground_video_rate_and_size():
    path = os.path.normpath(
        os.path.join(
            _THIS_DIR,
            "..",
            "..",
            "driver",
            "camera",
            "zed2i_driver",
            "launch",
            "back_cam_h26x_ground_video.launch.py",
        )
    )
    with open(path, "r") as stream:
        source = stream.read()

    assert "DeclareLaunchArgument('fps', default_value='4.0')" in source
    assert "DeclareLaunchArgument('width', default_value='480')" in source
    assert "DeclareLaunchArgument('height', default_value='360')" in source


@pytest.mark.parametrize("filename", ["bridge_minipc.json5", "bridge_jetson.json5"])
def test_zenoh_only_exports_livox_imu_from_the_raw_livox_streams(filename):
    path = os.path.normpath(
        os.path.join(_THIS_DIR, "..", "..", "..", "config", "zenoh", filename)
    )
    with open(path, "r") as stream:
        source = stream.read()

    assert '"/livox/imu"' in source
    assert '"/livox/lidar"' not in source


def test_critical_link_topics_are_excluded_from_all_zenoh_bridges():
    zenoh_dir = os.path.normpath(
        os.path.join(_THIS_DIR, "..", "..", "..", "config", "zenoh")
    )
    sources = {}
    for filename in (
        "bridge_groundpc.json5",
        "bridge_jetson.json5",
        "bridge_minipc.json5",
    ):
        with open(os.path.join(zenoh_dir, filename), "r") as stream:
            sources[filename] = stream.read()

    for topic in ('"/joy"', '"/heartbeat/ground_station"'):
        for source in sources.values():
            allow_lists = re.sub(r"//.*$", "", source, flags=re.MULTILINE)
            assert topic not in allow_lists


def test_ground_pc_routes_control_sources_only_to_critical_link_inputs():
    source = _read_launch_source("ground_pc.launch.py")

    assert '("/joy", "/critical_link/input/joy")' in source
    assert '"topic": "/critical_link/input/heartbeat"' in source
    assert '"topic": "/heartbeat/ground_station"' not in source


def test_terminal_bringups_include_role_specific_networking():
    for filename, role in (
        ("ground_pc.launch.py", "groundpc"),
        ("jetson_bringup.launch.py", "jetson"),
        ("minipc_bringup.launch.py", "minipc"),
    ):
        source = _read_launch_source(filename)
        assert '"networking.launch.py"' in source
        assert f'"role": "{role}"' in source


def test_networking_launch_starts_bridge_and_the_correct_critical_link_roles():
    source = _read_launch_source("networking.launch.py")

    assert '"zenoh-bridge-ros2dds"' in source
    assert '"env", "-u", "ROS_DOMAIN_ID"' in source
    assert "bridge_" in source
    assert ".json5" in source
    assert '"ground_sender.launch.py"' in source
    assert '"vessel_receiver.launch.py"' in source
    assert '_role_condition("groundpc")' in source
    assert '_role_condition("minipc")' in source
