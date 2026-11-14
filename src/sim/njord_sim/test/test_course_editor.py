import copy
import json
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python import packages
from mission_manager import course_editor_node
from njord_sim.scenario import scenario
from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.srv import SetParametersAtomically

from control_interfaces.msg import ControlState
from mission_interfaces.msg import MissionStatus


def test_sim_editor_save_guards_persistence_and_restart_geometry(tmp_path, monkeypatch):
    path = tmp_path / "config/course_overrides.yaml"
    path.parent.mkdir()
    path.write_text("tasks: {}\n")
    original_share = packages.get_package_share_directory

    def share(package):
        return str(tmp_path) if package == "njord_sim" else original_share(package)

    monkeypatch.setattr(packages, "get_package_share_directory", share)
    monkeypatch.setattr(course_editor_node, "share", share)
    rclpy.init(args=["--ros-args", "-p", "sim_task:=task1"], domain_id=121)
    editor = course_editor_node.CourseEditor()

    def apply(edit):
        request = SetParametersAtomically.Request(
            parameters=[
                Parameter(
                    name="edit",
                    value=ParameterValue(type=4, string_value=json.dumps(edit)),
                )
            ]
        )
        return editor._apply(request, SetParametersAtomically.Response()).result

    try:
        initial = copy.deepcopy(editor.state)
        edited = copy.deepcopy(initial)
        edited.update(action="save")
        edited["waypoints"][1]["latitude"] += 0.00001
        edited["buoys"][0]["type"] = "W"
        edited["buoys"][0]["longitude"] += 0.00001
        assert not apply(edited).successful  # No safety state received.
        assert yaml.safe_load(path.read_text()) == {"tasks": {}}
        editor._mission(MissionStatus(state=0))
        editor._control(ControlState(requested_mode=1))
        assert not apply(edited).successful
        editor._control(ControlState(requested_mode=0))
        editor._mission(MissionStatus(state=5))
        assert not apply(edited).successful
        editor._mission(MissionStatus(state=0))
        editor.control_at = time.monotonic() - 4
        assert not apply(edited).successful
        editor._control(ControlState(requested_mode=0))
        assert apply(edited).successful
        loaded = scenario("task1")
        assert loaded.route.waypoints[1].latitude == edited["waypoints"][1]["latitude"]
        assert loaded.buoys[0][2] == "W"
        assert editor.state["revision"] != initial["revision"]
        assert (
            editor.state["waypoints"][1]["latitude"]
            == edited["waypoints"][1]["latitude"]
        )
        # A different client retaining the old revision cannot overwrite changes.
        assert not apply(edited).successful
        assert (
            yaml.safe_load(path.read_text())["tasks"]["task1"]["buoys"][0]["type"]
            == "W"
        )
    finally:
        editor.destroy_node()
        rclpy.shutdown()


def test_real_yaml_edit_round_trip_over_ros_service(tmp_path, monkeypatch):
    from mission_manager.waypoint_config import WaypointConfigLoader
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node

    original_share = packages.get_package_share_directory
    path = tmp_path / "config/task1_waypoints.yaml"
    path.parent.mkdir()
    path.write_text(
        (
            Path(original_share("waypoint_publisher")) / "config/task1_waypoints.yaml"
        ).read_text()
    )
    monkeypatch.setattr(
        course_editor_node,
        "share",
        lambda package: (
            str(tmp_path)
            if package == "waypoint_publisher"
            else original_share(package)
        ),
    )
    rclpy.init(domain_id=122)
    editor = course_editor_node.CourseEditor()
    observer = Node("course_editor_service_test")
    executor = SingleThreadedExecutor()
    executor.add_node(editor)
    executor.add_node(observer)
    try:
        before = WaypointConfigLoader().load(path, "task1_config")
        edit = copy.deepcopy(editor.state)
        edit.update(action="save")
        edit["waypoints"][1]["latitude"] += 0.00001
        edit["buoys"] = [
            {"id": "b1", "type": "GREEN", "latitude": 34.5, "longitude": 135.5}
        ]
        editor._mission(MissionStatus(state=0))
        editor._control(ControlState(requested_mode=0))
        client = observer.create_client(SetParametersAtomically, "/course_editor/apply")
        assert client.wait_for_service(timeout_sec=3)
        request = SetParametersAtomically.Request(
            parameters=[
                Parameter(
                    name="edit",
                    value=ParameterValue(type=4, string_value=json.dumps(edit)),
                )
            ]
        )
        future = client.call_async(request)
        deadline = time.monotonic() + 3
        while not future.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert future.done() and future.result().result.successful
        after = WaypointConfigLoader().load(path, "task1_config")
        assert after.waypoints[1].latitude == edit["waypoints"][1]["latitude"]
        assert after.waypoints[1].yaw == before.waypoints[1].yaw
        assert after.constraints == before.constraints
        assert (
            yaml.safe_load(path.read_text())["map_buoys"]["task1_config"][0]["type"]
            == "GREEN"
        )
        assert "# Task 1 route:" in path.read_text()
    finally:
        executor.shutdown()
        editor.destroy_node()
        observer.destroy_node()
        rclpy.shutdown()
