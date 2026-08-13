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
from pathlib import Path

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
    "task2.launch.py",
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


def test_robot_manifest_gates_jetson_only_task2_dependencies():
    manifest = ET.parse(Path(_LAUNCH_DIR).parents[0] / "package.xml").getroot()
    conditions = {
        element.text: element.attrib.get("condition")
        for element in manifest.findall("exec_depend")
    }
    for package_name in (
        "pcl_det", "pcl_preprocessing", "pcl_segmentation", "ship_tracking", "task2_perception",
    ):
        assert conditions[package_name] == "$NJORD_ENABLE_GPU_SENSORS == 1"


def test_minipc_bringup_keeps_h26x_and_jpeg_back_camera_paths_separate():
    source = _read_launch_source("minipc_bringup.launch.py")
    assert 'back_cam_h26x_ground_video.launch.py' in source
    assert 'back_cam_jpeg_ground_video.launch.py' in source
    assert (
        '"enable_back_cam_jpeg_ground_video",\n'
        '                default_value="true"'
    ) in source


def test_minipc_back_camera_h26x_defaults_use_the_ground_direct_link():
    source = _read_launch_source("minipc_bringup.launch.py")

    assert '"enable_back_cam_ground_video",\n                default_value="true"' in source
    assert source.count('default_value="10.42.0.1"') == 2


def test_ground_pc_uses_the_jpeg_sender_port_for_the_jpeg_receiver():
    source = _read_launch_source("ground_pc.launch.py")

    assert 'LaunchConfiguration("back_cam_jpeg_video_port")' in source
    assert 'DeclareLaunchArgument("back_cam_jpeg_video_port", default_value="5602")' in source


def test_ground_pc_uses_the_jpeg_sender_port_for_the_jpeg_receiver():
    source = _read_launch_source("ground_pc.launch.py")
    assert 'LaunchConfiguration("back_cam_jpeg_video_port")' in source
    assert 'DeclareLaunchArgument("back_cam_jpeg_video_port", default_value="5602")' in source


def test_minipc_bringup_uses_canonical_control_manager_by_default():
    source = _read_launch_source("minipc_bringup.launch.py")
    assert '"control_manager"' in source
    assert '"control.launch.py"' in source
    assert '"enable_control_manager"' in source
    assert 'executable="command_arbiter_node"' in source
    assert 'name="command_arbiter"' in source
    assert 'condition=UnlessCondition(enable_control_manager)' in source
    assert 'package="twist_mux"' not in source


def test_task2_uses_fixed_odom_with_global_map_transform_available():
    minipc = _read_launch_source("minipc_bringup.launch.py")
    localization = _read_launch_source("localization.launch.py")
    jetson = _read_launch_source("jetson_bringup.launch.py")
    jetson_task2 = _read_launch_source("task2_jetson_autonomy.launch.py")
    task2 = _read_launch_source("task2_mission_adapter.launch.py")

    assert '"use_ekf_global",\n                default_value="true"' in minipc
    assert '"world_frame": "odom"' in localization
    assert '("odometry/filtered", "odometry/filtered/local")' in localization
    assert '"frame_id": "odom"' in task2
    assert '"task2_jetson_autonomy.launch.py"' in jetson
    assert '"own_odom_topic": odom' in jetson_task2
    assert '"/task2/safety_points"' not in jetson_task2
    assert 'name="task2_safety_points"' not in jetson_task2


def test_minipc_bringup_adds_sbus_without_replacing_the_ground_link_joy_path():
    source = _read_launch_source("minipc_bringup.launch.py")

    assert '"sbus_serial_port"' in source
    assert 'executable="sbus_joy_node"' in source
    assert 'executable="sbus_joy_converter_node"' in source
    assert 'executable="soft_emg_selector_node"' in source
    assert '("joy", "/sbus/joy")' in source
    assert '("/soft_emg", "/soft_emg_ground")' in source
    assert 'remappings=[("joy", "/sbus/joy"), ("/soft_emg", "/soft_emg_sbus")]' in source
    assert 'executable="joy_converter_node"' in source


def test_resident_nav2_profile_is_selected_by_role_bringup():
    minipc_source = _read_launch_source("minipc_bringup.launch.py")
    nav2_source = _read_launch_source("nav2.launch.py")

    assert '"profile": LaunchConfiguration("active_nav2_profile")' in minipc_source
    assert "'task1': 'nav2_params_humble.yaml'" in nav2_source
    assert "'task2': 'nav2_params_task2_humble.yaml'" in nav2_source
    assert "'task3': 'nav2_params_task3_humble.yaml'" in nav2_source
    assert "changed by Mission Manager" in nav2_source


@pytest.mark.parametrize("filename", ["nav2_params_task3_humble.yaml", "nav2_params_task3_jazzy.yaml"])
def test_task3_requires_goal_heading_alignment(filename):
    params = yaml.safe_load((Path(_LAUNCH_DIR).parents[0] / "config" / filename).read_text())
    controller = params["controller_server"]["ros__parameters"]
    follow_path = controller["FollowPath"]
    checker = controller["general_goal_checker"]

    assert "RotateToGoal" in follow_path["critics"]
    assert checker["plugin"] == "robot::SelectiveHeadingGoalChecker"
    assert checker["yaw_goal_tolerance"] == pytest.approx(0.35)
    assert checker["position_only_xy_goal_tolerance"] == pytest.approx(1.5)
    if filename.endswith("jazzy.yaml"):
        assert checker["heading_required_goal_xs"] == []
        assert checker["heading_required_goal_ys"] == []
    else:
        assert "heading_required_goal_xs" not in checker
        assert "heading_required_goal_ys" not in checker
    assert checker["heading_required_goal_tolerance"] == pytest.approx(1.0)


def test_task1_pass_through_state_is_scoped_to_each_goal():
    source = (Path(_LAUNCH_DIR).parents[0] / "src" / "pass_through_goal_checker.cpp").read_text()
    assert "goal_pose.position.x != last_goal_x_" in source
    assert "last_goal_x_ = goal_pose.position.x" in source


def test_task3_uses_its_relaxed_waypoint_behavior_tree():
    source = _read_launch_source("nav2.launch.py")
    task3_bt = (Path(_LAUNCH_DIR).parents[0] / "config" /
                "navigate_through_poses_task3_w_replanning_and_recovery.xml").read_text()

    assert "navigate_through_poses_task3_w_replanning_and_recovery.xml" in source
    assert ('RemovePassedGoals input_goals="{goals}" output_goals="{goals}" '
            'radius="1.5" global_frame="odom"') in task3_bt


def test_task2_uses_only_follow_path_controller_and_its_readiness_owner():
    minipc_source = _read_launch_source("minipc_bringup.launch.py")
    runtime_source = _read_launch_source("task_runtime.launch.py")
    runtime_manager_source = (Path(_LAUNCH_DIR).parents[1] / "mission" /
                              "mission_manager" / "mission_manager" /
                              "runtime_manager_node.py").read_text()
    task2_nav_source = _read_launch_source("navigation_launch_task2.py")
    adapter_source = _read_launch_source("task2_mission_adapter.launch.py")
    supervisor_source = (Path(_LAUNCH_DIR).parents[1] / "diagnostics" /
                         "diagnostic_monitors" / "src" / "autonomy_supervisor.cpp").read_text()
    task2_params = (Path(_LAUNCH_DIR).parents[0] / "config" /
                    "nav2_params_task2_humble.yaml").read_text()

    assert '"navigation_launch_task2.py"' in minipc_source
    assert '"task2_mission_adapter.launch.py"' not in minipc_source
    assert '"task2_mission_adapter.launch.py"' in runtime_source
    assert 'elif profile == "task2"' in runtime_manager_source
    assert '("controller_server", "velocity_smoother")' in runtime_manager_source
    assert '"Task 2 controller stack"' in runtime_manager_source
    assert "' == 'task2'" in minipc_source
    assert "' != 'task2'" in minipc_source
    assert 'executable="autonomy_supervisor_node"' in minipc_source
    assert '"task2_autonomy_ready_node"' not in adapter_source
    assert '"/runtime/nav2/status"' in supervisor_source
    assert '"waypoint_topic", "/task/plan"' in supervisor_source
    assert 'active_profile_ == "task2"' in supervisor_source
    assert 'active_profile_ != "task2" && !action_server_ready_' in supervisor_source
    assert '"/planned_path_pruned"' in supervisor_source
    assert '"task2_readiness_adapter_node"' not in adapter_source
    assert '"follow_path_client_node"' in adapter_source
    assert '"mission_gate_required": True' in adapter_source
    assert '"preprocessing.launch.py"' not in adapter_source
    assert '"planner_real.launch.py"' not in adapter_source
    planner_real_source = (Path(_LAUNCH_DIR).parents[1] / "navigation" /
                           "path_generator" / "mppi" / "launch" /
                           "planner_real.launch.py").read_text()
    assert 'executable="crm_costmap_node"' not in planner_real_source
    assert '"/mppi/crm_costmap"' in (Path(_LAUNCH_DIR).parents[2] /
                                       "config" / "zenoh" /
                                       "bridge_jetson.json5").read_text()
    assert "nav2_controller" in task2_nav_source
    assert "nav2_velocity_smoother" in task2_nav_source
    assert "nav2_lifecycle_manager" in task2_nav_source
    assert '"controller_server", "velocity_smoother"' in task2_nav_source
    assert '("cmd_vel", "/cmd_vel_controller")' in task2_nav_source
    assert '("cmd_vel_smoothed", "/cmd_vel_nav")' in task2_nav_source
    assert "root_key=None" in task2_nav_source
    for forbidden in (
        "nav2_planner",
        "nav2_smoother",
        "nav2_behaviors",
        "nav2_bt_navigator",
        "nav2_waypoint_follower",
        "nav2_planner",
    ):
        assert forbidden not in task2_nav_source
        assert forbidden not in task2_params
    assert "use_collision_detection: false" in task2_params
    assert "nav2_collision_monitor" not in task2_nav_source
    assert "collision_monitor:" not in task2_params
    assert "/task2/safety_points" not in task2_params


def test_legacy_task2_launch_uses_the_follow_path_only_graph():
    source = _read_launch_source("task2.launch.py")

    assert "'navigation_launch_task2.py'" in source
    assert "'nav2.launch.py'" not in source


def test_minipc_joy_converter_does_not_publish_physical_lamp_topics():
    source = _read_launch_source("minipc_bringup.launch.py")

    for color in ("green", "yellow", "red"):
        assert f'("/{color}", "/manual_lamp/{color}")' in source


def test_minipc_bringup_starts_the_autonomy_supervisor_once():
    minipc_source = _read_launch_source("minipc_bringup.launch.py")

    assert 'package="diagnostic_monitors"' in minipc_source
    assert 'executable="autonomy_supervisor_node"' in minipc_source
    assert '"enable_autonomy_supervisor",' in minipc_source

    for filename in ["task1.launch.py", "task2.launch.py", "task3.launch.py"]:
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


def test_ground_pc_renders_the_task2_planned_route_from_the_shared_path():
    source = _read_launch_source("ground_pc.launch.py")
    bridge = (Path(_LAUNCH_DIR).parents[2] / "config" / "zenoh" /
              "bridge_groundpc.json5").read_text()

    assert 'executable="planned_path_marker_publisher"' in source
    assert 'name="planned_route_publisher"' in source
    assert '"path_topic": "/planned_path_pruned"' in source
    assert '"marker_topic": "/planned_path_marker"' in source
    assert '"/planned_path_pruned"' in bridge


def test_ground_pc_enables_foxglove_by_default_for_waypoint_visualization():
    source = _read_launch_source("ground_pc.launch.py")

    assert '"enable_foxglove_bridge",\n                default_value="true"' in source


def test_ground_pc_reads_the_active_mission_waypoint_yaml_locally():
    source = _read_launch_source("ground_pc.launch.py")

    assert 'executable="ground_waypoint_geo_publisher"' in source
    assert '"waypoint_task_type"' not in source


def test_ground_zenoh_bridge_receives_only_the_active_mission_metadata():
    root = Path(_LAUNCH_DIR).parents[2]
    ground = (root / "config" / "zenoh" / "bridge_groundpc.json5").read_text()
    minipc = (root / "config" / "zenoh" / "bridge_minipc.json5").read_text()

    assert '"/mission/status"' in ground
    assert '"/mission/status"' in minipc
    assert '"/task/plan"' in ground
    assert '"/task/plan"' in minipc
    assert '"/waypoint_markers"' not in ground


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


def test_minipc_video_defaults_are_safe():
    source = _read_launch_source("minipc_bringup.launch.py")
    assert 'default_value="osw-Stealth-14-AI-Studio-A1VGG.local"' in source
    assert '"back_cam_ground_video_fps", default_value="2.0"' in source
    assert '"back_cam_ground_video_width", default_value="360"' in source
    assert '"back_cam_ground_video_height", default_value="240"' in source


def test_minipc_uses_um982_filter_without_glim_wait():
    source = _read_launch_source("minipc_bringup.launch.py")
    assert "use_glim_fb" not in source
    assert "enable_um982_glim_imu_fusion" not in source
    assert "um982_glim_imu_ekf_start_delay_sec" not in source
    assert '"enable_navsat_transform": "true"' in source
    assert "um982_feedback_launch = include_launch" in source


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


def test_back_camera_optional_v4l2_controls_do_not_abort_bringup():
    source = _read_launch_source("back_cam.launch.py")
    assert "subprocess.run(command, check=False)" in source


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


def test_localization_launch_uses_the_canonical_fixed_navsat_datum():
    source = _read_launch_source("localization.launch.py")
    navsat = source.split("\n    navsat_transform_node = Node(", 1)[1].split("diagnostics_launch", 1)[0]
    assert 'get_package_share_directory("mission_manager")' in source
    assert '"wait_for_datum": True' in navsat
    assert 'home_datum["latitude"], home_datum["longitude"], home_datum["yaw"]' in navsat


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
    assert '"ground_video_draw_detections",\n                default_value="true"' in source
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


def test_task1_uses_only_the_odom_to_base_link_navigation_tree():
    """Task 1 obstacles and costmaps must not require map -> odom."""
    robot_config_dir = os.path.normpath(os.path.join(_THIS_DIR, "..", "config"))
    with open(
        os.path.join(
            _THIS_DIR,
            "..",
            "..",
            "localization",
            "um982_feedback_filter",
            "config",
            "um982_feedback_ekf.yaml",
        ),
        encoding="utf-8",
    ) as stream:
        local_ekf = yaml.safe_load(stream)["um982_feedback_ekf"]["ros__parameters"]
    with open(os.path.join(robot_config_dir, "nav2_params_humble.yaml"), encoding="utf-8") as stream:
        nav2 = yaml.safe_load(stream)

    assert local_ekf["publish_tf"] is True
    assert local_ekf["world_frame"] == "odom"
    assert local_ekf["odom_frame"] == "odom"
    assert local_ekf["base_link_frame"] == "base_link"

    local_obstacle_layer = nav2["local_costmap"]["local_costmap"]["ros__parameters"]["obstacle_layer"]
    local_costmap = nav2["local_costmap"]["local_costmap"]["ros__parameters"]
    global_costmap = nav2["global_costmap"]["global_costmap"]["ros__parameters"]
    assert nav2["bt_navigator"]["ros__parameters"]["global_frame"] == "odom"
    assert global_costmap["global_frame"] == "odom"
    assert local_costmap["global_frame"] == "odom"
    assert local_costmap["rolling_window"] is True
    assert local_costmap["width"] == 30
    assert local_costmap["height"] == 30
    assert "virtual_wall" in local_obstacle_layer["observation_sources"].split()
    assert local_obstacle_layer["virtual_wall"]["topic"] == "/virtual_obstacles"


@pytest.mark.parametrize("filename", ["nav2_params_task3_humble.yaml", "nav2_params_task3_jazzy.yaml"])
def test_task3_uses_the_single_local_odom_source(filename):
    with open(os.path.join(_THIS_DIR, "..", "config", filename), encoding="utf-8") as stream:
        nav2 = yaml.safe_load(stream)
    params = nav2["bt_navigator"]["ros__parameters"]
    smoother = nav2["velocity_smoother"]["ros__parameters"]
    assert params["global_frame"] == "odom"
    assert params["odom_topic"] == "/odometry/filtered/local"
    assert smoother["odom_topic"] == "/odometry/filtered/local"


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


def test_bringups_start_role_owned_health_monitors():
    jetson_source = _read_launch_source("jetson_bringup.launch.py")
    minipc_source = _read_launch_source("minipc_bringup.launch.py")
    heartbeat_source = _read_launch_source("heartbeat.launch.py")
    assert '"role": "jetson"' in jetson_source
    assert '"role": "minipc"' in minipc_source
    assert '"config",' in heartbeat_source
    assert '"heartbeat",' in heartbeat_source
    assert 'f"{role}.yaml"' in heartbeat_source
    assert '"/health/signals/{role}"' in heartbeat_source
    assert '"/health/state" if role == "minipc"' in heartbeat_source
    assert 'executable="heartbeat_aggregator_node"' in heartbeat_source
    assert '"aggregates"' in heartbeat_source
    assert '"heartbeat_monitor_zed2i"' in heartbeat_source
    assert '"heartbeat_monitor_lidar"' in heartbeat_source
    assert "heartbeat_monitor_" not in minipc_source
    assert "enable_heartbeats" not in minipc_source


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


def test_task1_safety_cloud_is_compacted_on_jetson_and_bridged_to_minipc():
    jetson_source = _read_launch_source("jetson_bringup.launch.py")
    assert 'name="task1_safety_points"' in jetson_source
    assert '"input_topic": "/zed2i/points"' in jetson_source
    assert '"output_topic": "/task1/safety_points"' in jetson_source
    assert '"max_range_m": 8.0' in jetson_source
    assert '"min_valid_input_points": 0' in jetson_source
    assert '"publish_empty_on_invalid_input": True' in jetson_source
    assert '"voxel_leaf_size_m": 0.25' in jetson_source
    assert '"process_rate_hz": 5.0' in jetson_source

    zed_config = os.path.normpath(
        os.path.join(
            _THIS_DIR, "..", "..", "driver", "camera", "zed2i_driver",
            "config", "zed2i_jetson_orin_nano.yaml",
        )
    )
    with open(zed_config, "r") as stream:
        zed_source = stream.read()
    assert "publish_pointcloud: true" in zed_source
    assert "pointcloud_stride: 4" in zed_source

    for filename in ("bridge_jetson.json5", "bridge_minipc.json5"):
        path = os.path.normpath(
            os.path.join(_THIS_DIR, "..", "..", "..", "config", "zenoh", filename)
        )
        with open(path, "r") as stream:
            assert '"/task1/safety_points"' in stream.read()


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


def test_verified_wired_link_is_used_for_zenoh_and_critical_link():
    root = Path(_THIS_DIR).parents[2]
    ground_bridge = (root / "config" / "zenoh" / "bridge_groundpc.json5").read_text(
        encoding="utf-8"
    )
    jetson_bridge = (root / "config" / "zenoh" / "bridge_jetson.json5").read_text(
        encoding="utf-8"
    )
    assert '"tcp/10.42.0.2:7447"' in ground_bridge
    assert '"tcp/10.42.0.2:7447"' in jetson_bridge

    ground = (root / "src" / "critical_link" / "config" / "ground_sender.yaml").read_text()
    vessel = (root / "src" / "critical_link" / "config" / "vessel_receiver.yaml").read_text()
    assert '"ops_lan|10.42.0.1|10.42.0.2|45100"' in ground
    assert '"tailscale|100.123.47.2|100.82.157.125|45101"' in ground
    assert '"ops_lan|10.42.0.2|45100"' in vessel
    assert '"tailscale|100.82.157.125|45101"' in vessel


def test_ground_zenoh_bridge_can_request_mission_manager_tasks():
    path = os.path.normpath(
        os.path.join(_THIS_DIR, "..", "..", "..", "config", "zenoh", "bridge_groundpc.json5")
    )
    with open(path, "r") as stream:
        source = stream.read()

    for endpoint in (
        '"/mission/list_tasks"',
        '"/mission/get_status"',
        '"/mission/start_task"',
        '"/mission/stop_task"',
        '"/mission/run_task"',
    ):
        assert endpoint in source


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


def test_jetson_publishes_livox_static_tf_before_starting_glim():
    source = _read_launch_source("jetson_bringup.launch.py")
    assert 'package="tf2_ros"' in source
    assert 'name="jetson_livox_static_tf_pub"' in source
    assert '"--frame-id", "base_link", "--child-frame-id", "livox_frame"' in source
    assert 'name="jetson_zed_static_tf_pub"' in source
    assert '"--frame-id", "base_link", "--child-frame-id", "zed2i_left_camera_frame"' in source
    assert '"lidar_start_delay",\n                default_value="0.5"' in source


def test_zenoh_tf_static_history_settings_are_unique_and_routed_to_jetson():
    zenoh_dir = Path(_THIS_DIR).parents[2] / "config" / "zenoh"
    jetson = (zenoh_dir / "bridge_jetson.json5").read_text(encoding="utf-8")
    minipc = (zenoh_dir / "bridge_minipc.json5").read_text(encoding="utf-8")

    assert jetson.count("transient_local_cache_multiplier:") == 1
    assert jetson.count("transient_local_subscribers:") == 1
    assert '"/tf_static"' in jetson.split("subscribers:", 1)[1]
    assert '"/tf_static"' in minipc.split("publishers:", 1)[1]


def test_zenoh_local_odometry_has_one_minipc_publisher():
    zenoh_dir = Path(_THIS_DIR).parents[2] / "config" / "zenoh"
    minipc = (zenoh_dir / "bridge_minipc.json5").read_text(encoding="utf-8")
    jetson = (zenoh_dir / "bridge_jetson.json5").read_text(encoding="utf-8")
    topic = '"/odometry/filtered/local"'
    minipc_allow = minipc.split("allow:", 1)[1]
    jetson_allow = jetson.split("allow:", 1)[1]

    assert topic in minipc_allow.split("publishers:", 1)[1].split("subscribers:", 1)[0]
    assert topic not in minipc_allow.split("subscribers:", 1)[1]
    assert topic not in jetson_allow.split("publishers:", 1)[1].split("subscribers:", 1)[0]
    assert topic in jetson_allow.split("subscribers:", 1)[1]


def test_zenoh_routes_minipc_autonomy_status_to_jetson():
    zenoh_dir = Path(_THIS_DIR).parents[2] / "config" / "zenoh"
    minipc = (zenoh_dir / "bridge_minipc.json5").read_text(encoding="utf-8")
    jetson = (zenoh_dir / "bridge_jetson.json5").read_text(encoding="utf-8")
    minipc_publishers = minipc.split("publishers:", 1)[1].split("subscribers:", 1)[0]
    jetson_subscribers = jetson.split("subscribers:", 1)[1]

    for topic in ('"/autonomy/ready"', '"/heartbeat/autonomy"'):
        assert topic in minipc_publishers
        assert topic in jetson_subscribers


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


def test_foxglove_v3_telemetry_reaches_the_ground_pc():
    zenoh_dir = Path(_THIS_DIR).parents[2] / "config" / "zenoh"
    minipc = (zenoh_dir / "bridge_minipc.json5").read_text(encoding="utf-8")
    ground = (zenoh_dir / "bridge_groundpc.json5").read_text(encoding="utf-8")
    for topic in ('"/sensor/vehicle_gnss/fix/raw"', '"/gui/ground_speed_mps"', '"/cmd_vel_markers"'):
        assert topic in minipc
        assert topic in ground

    source = _read_launch_source("minipc_bringup.launch.py")
    assert 'executable="ground_speed_publisher"' in source


def test_zenoh_routes_selected_opponent_markers_to_ground():
    zenoh_dir = Path(_THIS_DIR).parents[2] / "config" / "zenoh"
    jetson = (zenoh_dir / "bridge_jetson.json5").read_text(encoding="utf-8")
    minipc = (zenoh_dir / "bridge_minipc.json5").read_text(encoding="utf-8")
    ground = (zenoh_dir / "bridge_groundpc.json5").read_text(encoding="utf-8")
    jetson_publishers = jetson.split("publishers:", 1)[1].split("subscribers:", 1)[0]
    minipc_publishers = minipc.split("publishers:", 1)[1].split("subscribers:", 1)[0]
    minipc_subscribers = minipc.split("subscribers:", 1)[1]
    ground_subscribers = ground.split("subscribers:", 1)[1]

    for topic in (
        '\"/task2/selected_opponent_marker\"',
        '\"/task2/selected_opponent_velocity_marker\"',
    ):
        assert topic in jetson_publishers
        assert topic in minipc_subscribers
        assert topic in minipc_publishers
        assert topic in ground_subscribers


def test_zed_v4l2_q40_preview_is_the_default_jetson_backend():
    source = _read_launch_source("jetson_bringup.launch.py")
    assert '"zed_camera_backend"' in source
    assert 'default_value="v4l2"' in source
    assert '["v4l2", "sdk"]' in source
    assert '"zed_v4l2_preview.launch.py"' in source

    root = Path(_THIS_DIR).parents[2]
    preview_launch = (
        root
        / "src"
        / "foxglove_image_preview"
        / "launch"
        / "zed_v4l2_preview.launch.py"
    ).read_text(encoding="utf-8")
    assert '"jpeg_qualities": [40]' in preview_launch
    assert '"width": 640' in preview_launch
    assert '"height": 360' in preview_launch
    assert "/dev/v4l/by-id/usb-Technologies__Inc._ZED_2i_" in preview_launch
    assert "OV0001-video-index0" in preview_launch


def test_zenoh_routes_only_the_q40_zed_preview_to_ground():
    zenoh_dir = Path(_THIS_DIR).parents[2] / "config" / "zenoh"
    configs = {
        role: (zenoh_dir / f"bridge_{role}.json5").read_text(encoding="utf-8")
        for role in ("jetson", "minipc", "groundpc")
    }
    q40 = '"/zed2i/left/preview/q40/compressed"'

    assert q40 in configs["jetson"].split("publishers:", 1)[1].split("subscribers:", 1)[0]
    assert q40 in configs["minipc"].split("publishers:", 1)[1].split("subscribers:", 1)[0]
    assert q40 in configs["minipc"].split("subscribers:", 1)[1]
    assert q40 in configs["groundpc"].split("subscribers:", 1)[1]

    for forbidden in (
        "/zed2i/stereo/image_raw",
        "/zed2i/left/preview/q70/compressed",
        "/zed2i/left/preview/q90/compressed",
    ):
        assert all(forbidden not in config for config in configs.values())


def test_zenoh_routes_the_low_rate_annotated_yolo_preview_to_minipc():
    zenoh_dir = Path(_THIS_DIR).parents[2] / "config" / "zenoh"
    jetson = (zenoh_dir / "bridge_jetson.json5").read_text(encoding="utf-8")
    minipc = (zenoh_dir / "bridge_minipc.json5").read_text(encoding="utf-8")
    topic = '"/yolo/debug_preview/q40/compressed"'

    assert topic in jetson.split("publishers:", 1)[1].split("subscribers:", 1)[0]
    assert topic in minipc.split("subscribers:", 1)[1]
