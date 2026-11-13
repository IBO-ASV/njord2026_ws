"""Map course editing over an existing typed ROS parameter transaction service."""

import json
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory as share
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from njord_interfaces.msg import ControlState, MissionStatus

from .course_editing import atomic_save, revision, validate_edit, waypoint_yaml_edit
from .task_registry import TaskRegistry
from .waypoint_config import WaypointConfigLoader

QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class CourseEditor(Node):
    def __init__(self):
        super().__init__("course_editor")
        self.sim_task = str(self.declare_parameter("sim_task", "").value)
        self.selected = self.sim_task or "task1"
        self.registry = TaskRegistry.from_file(
            Path(share("mission_manager")) / "config/task_registry.yaml"
        )
        self.mission = self.control = None
        self.mission_at = self.control_at = 0
        self.state = {}
        self.publisher = self.create_publisher(String, "/course_editor/state", QOS)
        self.create_subscription(MissionStatus, "/mission/status", self._mission, QOS)
        self.create_subscription(ControlState, "/control/state", self._control, QOS)
        self.create_service(
            SetParametersAtomically, "/course_editor/apply", self._apply
        )
        self._load()
        self.create_timer(1.0, self._publish)
        self._publish()

    def _mission(self, message):
        self.mission, self.mission_at = message, time.monotonic()
        if (
            not self.sim_task
            and 1 <= message.state <= 7
            and message.task_id in self.state.get("tasks", [])
            and message.task_id != self.selected
        ):
            self.selected = message.task_id
            try:
                self._load()
            except (OSError, ValueError, KeyError, yaml.YAMLError) as exc:
                self.get_logger().error(f"Cannot load active course: {exc}")

    def _control(self, message):
        self.control, self.control_at = message, time.monotonic()

    def _editable(self):
        return bool(
            self.mission is not None
            and self.control is not None
            and time.monotonic() - self.mission_at < 3
            and time.monotonic() - self.control_at < 3
            and self.mission.state == MissionStatus.STATE_IDLE
            and self.control.requested_mode == ControlState.MODE_MANUAL
            and self.control.effective_source != ControlState.SOURCE_AUTO
        )

    def _load(self):
        if self.sim_task:
            from njord_sim.scenario import inverse_enu, scenario

            scene = scenario(self.sim_task)
            self.path = (
                Path(share("njord_sim")) / "config/course_overrides.yaml"
            ).resolve()
            waypoints = [
                {
                    "id": w.waypoint_id,
                    "name": w.name,
                    "latitude": w.latitude,
                    "longitude": w.longitude,
                }
                for w in scene.route.waypoints
            ]
            buoys = [
                {
                    "id": f"buoy-{i}",
                    "latitude": inverse_enu(x, y, scene.datum)[0],
                    "longitude": inverse_enu(x, y, scene.datum)[1],
                    "type": kind,
                }
                for i, (x, y, kind) in enumerate(scene.buoys)
            ]
            saved = (
                yaml.safe_load(self.path.read_text())
                .get("tasks", {})
                .get(self.selected)
            )
            if saved:
                buoys = saved["buoys"]
            task_ids = [self.sim_task]
        else:
            task = self.registry.get(self.selected)
            if task is None or not task.route_package:
                raise ValueError("unknown editable task")
            self.path = (Path(share(task.route_package)) / task.route).resolve()
            route = WaypointConfigLoader().load(self.path, task.route_key)
            waypoints = [
                {
                    "id": w.waypoint_id,
                    "name": w.name,
                    "latitude": w.latitude,
                    "longitude": w.longitude,
                }
                for w in route.waypoints
            ]
            buoys = (
                yaml.safe_load(self.path.read_text())
                .get("map_buoys", {})
                .get(task.route_key, [])
            )
            task_ids = [
                t.task_id
                for t in self.registry.all()
                if t.route_package and t.task_id != "return_home"
            ]
        self.state = {
            "mode": "sim" if self.sim_task else "real",
            "task_id": self.selected,
            "tasks": task_ids,
            "source_file": str(self.path),
            "revision": revision(self.path),
            "waypoints": waypoints,
            "buoys": buoys,
            "requires_restart": bool(self.sim_task),
        }

    def _publish(self):
        try:
            if revision(self.path) != self.state["revision"]:
                self._load()
            state = {**self.state, "editable": self._editable()}
            self.publisher.publish(String(data=json.dumps(state, allow_nan=False)))
        except (OSError, ValueError, KeyError, yaml.YAMLError) as exc:
            self.publisher.publish(
                String(data=json.dumps({"editable": False, "error": str(exc)}))
            )

    def _apply(self, request, response):
        try:
            if (
                len(request.parameters) != 1
                or request.parameters[0].name != "edit"
                or request.parameters[0].value.type != 4
            ):
                raise ValueError("expected one string parameter named edit")
            if len(request.parameters[0].value.string_value) > 1000000:
                raise ValueError("edit request is too large")
            edit = json.loads(request.parameters[0].value.string_value)
            if not isinstance(edit, dict):
                raise ValueError("edit must be a JSON object")
            task_id = edit.get("task_id")
            if task_id not in self.state["tasks"]:
                raise ValueError("task is not available in this session")
            if edit.get("action") == "load":
                self.selected = task_id
                self._load()
                response.result.reason = "Loaded course"
            elif edit.get("action") == "save":
                if not self._editable():
                    raise ValueError(
                        "save requires fresh Mission IDLE and MANUAL control state"
                    )
                if (
                    task_id != self.selected
                    or edit.get("revision") != self.state["revision"]
                ):
                    raise ValueError("course changed; reload before saving")
                validate_edit(self.state, edit)
                text = self.path.read_text()
                if self.sim_task:
                    data = yaml.safe_load(text)
                    data.setdefault("tasks", {})[task_id] = {
                        "waypoints": [
                            {
                                **old,
                                "latitude": new["latitude"],
                                "longitude": new["longitude"],
                            }
                            for old, new in zip(
                                self.state["waypoints"], edit["waypoints"]
                            )
                        ],
                        "buoys": edit["buoys"],
                    }
                    text = yaml.safe_dump(data, sort_keys=False, allow_unicode=True)
                else:
                    task = self.registry.get(task_id)
                    text = waypoint_yaml_edit(
                        text, task.route_key, edit["waypoints"], edit["buoys"]
                    )
                    # Validate the edited route before replacing its source.
                    raw = yaml.safe_load(text)[task.route_key]
                    WaypointConfigLoader()._waypoints(raw["waypoints"], task.route_key)
                atomic_save(self.path, text, edit["revision"])
                self._load()
                response.result.reason = (
                    "YAML saved; restart sim to apply geometry"
                    if self.sim_task
                    else "YAML saved; next mission uses edited waypoints (buoys are map annotations)"
                )
            else:
                raise ValueError("unknown edit action")
            response.result.successful = True
        except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
            response.result.successful = False
            response.result.reason = str(exc)
        self._publish()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CourseEditor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
