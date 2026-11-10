"""Canonical action/service owner for a single NJORD mission execution."""

from __future__ import annotations

import math
import threading
from dataclasses import replace
from pathlib import Path
from typing import Callable, Optional, Sequence

import rclpy
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped
from geographic_msgs.msg import GeoPoint
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Path as NavPath
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from robot_localization.srv import FromLL
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from std_msgs.msg import Bool, Empty, Float64, String
from visualization_msgs.msg import Marker, MarkerArray
from waypoint_transformer import transform_waypoints

from njord_interfaces.action import ConfigureSystem, RunTask
from njord_interfaces.msg import ControlState, MissionStatus, Nav2RuntimeStatus, TaskInfo
from njord_interfaces.srv import GetMissionStatus, ListTasks, SetControlMode, StartTask, StopTask

from .executors import (
    DummyExecutor,
    ExecutorStatus,
    StagedDockingExecutor,
    Task2MppiExecutor,
    Task4CompositeExecutor,
    WaypointSequenceExecutor,
)
from .state_machine import MissionState, MissionStateMachine, ResultCode, StartRequest
from .task_registry import (
    RUNTIME_READINESS_FEATURES,
    RegistryError,
    TaskDefinition,
    TaskRegistry,
    required_runtime_readiness,
)
from .waypoint_config import GeodeticPoint, Route, Waypoint, WaypointConfigLoader


_STATUS_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class _RosNavigationClient:
    """Converts typed waypoints to Nav2 messages and keeps one active goal."""

    def __init__(self, node: Node, path_publisher, marker_publisher) -> None:
        self._node = node
        self._path_publisher = path_publisher
        self._marker_publisher = marker_publisher
        self._client = ActionClient(node, NavigateThroughPoses, "/navigate_through_poses")
        self._goal_handle = None
        self._callbacks = None
        self._frame_id = "map"
        self._feedback_callback = None

    def send(self, poses: Sequence[Waypoint], accepted, completed) -> None:
        self._callbacks = (accepted, completed)
        if not self._client.server_is_ready():
            accepted(False)
            return
        messages = [self._pose(waypoint) for waypoint in poses]
        self._publish_path(messages)
        self._publish_markers(messages)
        goal = NavigateThroughPoses.Goal()
        goal.poses = messages
        self._client.send_goal_async(goal, feedback_callback=self._on_feedback).add_done_callback(
            self._on_goal_response
        )

    def publish_preview(self, waypoints: Sequence[Waypoint]) -> None:
        """Publish a route visualization without sending a Nav2 goal."""
        poses = [self._pose(waypoint) for waypoint in waypoints]
        self._publish_path(poses)
        self._publish_markers(poses)

    def set_frame_id(self, frame_id: str) -> None:
        self._frame_id = frame_id

    def set_feedback_callback(self, callback) -> None:
        self._feedback_callback = callback

    def cancel(self) -> None:
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()

    def _pose(self, waypoint: Waypoint) -> PoseStamped:
        import math

        pose = PoseStamped()
        pose.header.frame_id = self._frame_id
        pose.header.stamp = self._node.get_clock().now().to_msg()
        pose.pose.position.x = waypoint.x
        pose.pose.position.y = waypoint.y
        pose.pose.orientation.z = math.sin(waypoint.yaw / 2.0)
        pose.pose.orientation.w = math.cos(waypoint.yaw / 2.0)
        return pose

    def _publish_path(self, poses: list[PoseStamped]) -> None:
        path = NavPath()
        path.header.frame_id = self._frame_id
        path.header.stamp = self._node.get_clock().now().to_msg()
        path.poses = poses
        self._path_publisher.publish(path)

    def _publish_markers(self, poses: list[PoseStamped]) -> None:
        if not poses:
            return
        stamp = self._node.get_clock().now().to_msg()
        markers = MarkerArray()
        route = Marker()
        route.header.frame_id, route.header.stamp = self._frame_id, stamp
        route.ns, route.id, route.type, route.action = (
            "mission_waypoint_route", 0, Marker.LINE_STRIP, Marker.ADD
        )
        route.pose.orientation.w, route.scale.x = 1.0, 0.12
        route.color.r, route.color.g, route.color.b, route.color.a = 0.1, 0.9, 1.0, 0.9
        points = Marker()
        points.header.frame_id, points.header.stamp = self._frame_id, stamp
        points.ns, points.id, points.type, points.action = (
            "mission_waypoints", 0, Marker.SPHERE_LIST, Marker.ADD
        )
        points.pose.orientation.w = 1.0
        points.scale.x = points.scale.y = points.scale.z = 0.35
        points.color.g, points.color.b, points.color.a = 0.95, 1.0, 1.0
        for pose in poses:
            point = Point(x=pose.pose.position.x, y=pose.pose.position.y, z=0.18)
            route.points.append(point)
            points.points.append(point)
        markers.markers = [route, points]
        self._marker_publisher.publish(markers)

    def _on_goal_response(self, future) -> None:
        callbacks = self._callbacks
        if callbacks is None:
            return
        try:
            self._goal_handle = future.result()
        except Exception as exc:
            callbacks[0](False)
            self._node.get_logger().error(f"Nav2 goal send failed: {exc}")
            return
        callbacks[0](self._goal_handle.accepted)
        if self._goal_handle.accepted:
            self._goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        callbacks = self._callbacks
        self._goal_handle = None
        if callbacks is None:
            return
        try:
            wrapped = future.result()
            if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
                callbacks[1](ExecutorStatus.SUCCEEDED, "Nav2 goal succeeded")
            elif wrapped.status == GoalStatus.STATUS_CANCELED:
                callbacks[1](ExecutorStatus.CANCELED, "Nav2 goal canceled")
            else:
                callbacks[1](ExecutorStatus.NAVIGATION_FAILED, f"Nav2 goal status={wrapped.status}")
        except Exception as exc:
            callbacks[1](ExecutorStatus.NAVIGATION_FAILED, f"Nav2 result failed: {exc}")

    def _on_feedback(self, feedback_msg) -> None:
        if self._feedback_callback is not None:
            self._feedback_callback(feedback_msg.feedback)


class MissionManager(Node):
    """Thread-safe adapter around the serialized :class:`MissionStateMachine`."""

    def __init__(self) -> None:
        super().__init__("mission_manager")
        self.declare_parameter("registry_file", "")
        self.declare_parameter("active_nav2_profile", "")
        self.declare_parameter("auto_permission_timeout_sec", 30.0)
        self.declare_parameter("coordinate_projection_timeout_sec", 30.0)
        self.declare_parameter("coordinate_projection_retry_sec", 0.5)
        self.declare_parameter("coordinate_projection_request_timeout_sec", 5.0)
        self.declare_parameter("waypoint_transform_enabled", False)
        self.declare_parameter("waypoint_transform_anchor_latitude", 0.0)
        self.declare_parameter("waypoint_transform_anchor_longitude", 0.0)
        self.declare_parameter("waypoint_transform_rotation_rad", 0.0)
        self.declare_parameter("waypoint_transform_scale", 1.0)
        self.declare_parameter("force_runtime_reconfigure", False)
        self.declare_parameter("skip_coordinate_projection", False)
        self._lock = threading.RLock()
        self._cb_group = ReentrantCallbackGroup()
        self._machine = MissionStateMachine()
        self._loader = WaypointConfigLoader()
        self._active_executor = None
        self._active_task: TaskDefinition | None = None
        self._active_route: Route | None = None
        self._pending_start = None
        self._pending_runtime_start = None
        self._active_nav2_profile = str(self.get_parameter("active_nav2_profile").value)
        self._runtime_state = Nav2RuntimeStatus.STOPPED
        self._pending_coordinate_projection = None
        self._coordinate_projection_deadline_ns: int | None = None
        self._pending_task3_goal_checker = None
        self._auto_permission_deadline_ns: int | None = None
        self._auto_mode_request_sent = False
        self._manual_mode_requested: set[str] = set()
        self._action_results: dict[str, tuple[ResultCode, str]] = {}
        self._action_tick = Future()
        self._auto_permitted = False
        self._requested_mode = MissionStatus.MODE_MANUAL
        self._effective_source = MissionStatus.SOURCE_ZERO
        self._inhibit_reasons: list[str] = ["control state unavailable"]
        self._last_transition = self.get_clock().now().to_msg()
        self._last_snapshot = self._machine.snapshot
        self._task_ready = False
        self._task_requirements_ready = False
        self._active_control_policy = ""
        self._runtime_readiness = {name: False for name in RUNTIME_READINESS_FEATURES}
        self._pending_failsafe_return = False
        self._failsafe_return_count = 0

        registry_path = self._registry_path()
        shares = {"waypoint_publisher": self._waypoint_share_path()}
        self._registry = TaskRegistry.from_file(registry_path, package_shares=shares)

        self._status_pub = self.create_publisher(MissionStatus, "/mission/status", _STATUS_QOS)
        self._plan_pub = self.create_publisher(NavPath, "/task/plan", _STATUS_QOS)
        self._waypoint_markers_pub = self.create_publisher(
            MarkerArray, "/mission/waypoint_markers", _STATUS_QOS
        )
        self._task_ready_pub = self.create_publisher(Bool, "/mission/task_ready", _STATUS_QOS)
        self._task_requirements_ready_pub = self.create_publisher(
            Bool, "/mission/task_requirements_ready", _STATUS_QOS
        )
        self._active_policy_pub = self.create_publisher(
            String, "/mission/active_control_policy", _STATUS_QOS
        )
        self._task2_enabled_pub = self.create_publisher(Bool, "/mission/task2/enabled", _STATUS_QOS)
        self.create_subscription(Bool, "/mission/task2/goal_reached", self._on_task2_goal_reached, _STATUS_QOS)
        self._task1_cardinal_wall_enable_pub = self.create_publisher(
            Bool, "/task1/cardinal_wall_enable", _STATUS_QOS
        )
        self._task1_cardinal_heading_pub = self.create_publisher(
            Float64, "/task1/gps3_to_gps4_heading", _STATUS_QOS
        )
        self._task4_buoy_wall_enable_pub = self.create_publisher(
            Bool, "/task4/buoy_wall_enable", _STATUS_QOS
        )
        self._task4_buoy_wall_heading_pub = self.create_publisher(
            Float64, "/task4/gps14_to_gps16_heading", _STATUS_QOS
        )
        self._task1_wall_enable_after_remaining: int | None = None
        self._task1_walls_enabled = False
        self._task4_walls_enabled = False
        self._set_task2_enabled(False)
        self._set_task1_cardinal_walls(False)
        self._set_task4_buoy_walls(False)
        self._navigation = _RosNavigationClient(
            self, self._plan_pub, self._waypoint_markers_pub
        )
        self._from_ll_client = self.create_client(FromLL, "/fromLL")
        self._task3_goal_checker_client = self.create_client(
            SetParameters, "/controller_server/set_parameters"
        )
        self._dock_nominal_pub = {
            berth: self.create_publisher(PoseStamped, f"/docking/{berth}_nominal", _STATUS_QOS)
            for berth in ("berth1", "berth2")
        }
        self._dock_corrections: dict[str, PoseStamped] = {}
        for berth in self._dock_nominal_pub:
            self.create_subscription(PoseStamped, f"/docking/{berth}_corrected",
                                     lambda message, berth=berth: self._on_dock_correction(berth, message), 10)
        self._control_sub = self.create_subscription(
            ControlState, "/control/state", self._on_control_state, _STATUS_QOS,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            Empty, "/mission/failsafe/return_home", self._on_failsafe_return, _STATUS_QOS,
            callback_group=self._cb_group,
        )
        for name in RUNTIME_READINESS_FEATURES:
            self.create_subscription(
                Bool,
                f"/mission/readiness/{name}",
                lambda message, readiness_name=name: self._on_runtime_readiness(
                    readiness_name, message
                ),
                _STATUS_QOS,
                callback_group=self._cb_group,
            )
        self._action = ActionServer(
            self, RunTask, "/mission/run_task", self._execute_action,
            goal_callback=self._goal_callback, cancel_callback=self._cancel_callback,
            callback_group=self._cb_group,
        )
        self._start_srv = self.create_service(StartTask, "/mission/start_task", self._on_start)
        self._stop_srv = self.create_service(StopTask, "/mission/stop_task", self._on_stop)
        self._list_srv = self.create_service(ListTasks, "/mission/list_tasks", self._on_list)
        self._status_srv = self.create_service(GetMissionStatus, "/mission/get_status", self._on_status)
        self._set_mode_client = self.create_client(SetControlMode, "/control/set_mode")
        self._configure_runtime_client = ActionClient(self, ConfigureSystem, "/system/configure")
        self.create_subscription(Nav2RuntimeStatus, "/runtime/nav2/status", self._on_runtime_status, _STATUS_QOS)
        self._timer = self.create_timer(0.1, self._tick, callback_group=self._cb_group)
        self._publish_task_readiness(False, False, "")
        self._publish_status()
        self.get_logger().info(f"Mission Manager ready with registry {registry_path}")

    def destroy_node(self):
        with self._lock:
            self._set_task2_enabled(False)
            if self._active_executor and self._machine.snapshot.execution_id:
                self._active_executor.cleanup(self._machine.snapshot.execution_id)
        return super().destroy_node()

    def _registry_path(self) -> Path:
        configured = str(self.get_parameter("registry_file").value)
        if configured:
            return Path(configured)
        return Path(get_package_share_directory("mission_manager")) / "config" / "task_registry.yaml"

    @staticmethod
    def _waypoint_share_path() -> Path:
        # Installed packages keep their resources in the ament share directory.
        return Path(get_package_share_directory("waypoint_publisher"))

    def _goal_callback(self, goal_request) -> GoalResponse:
        # Admission happens atomically in execute/start callbacks to preserve request-id semantics.
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    async def _execute_action(self, goal_handle):
        request = StartRequest(
            goal_handle.request.task_id,
            goal_handle.request.request_auto_mode,
            goal_handle.request.dry_run,
            goal_handle.request.request_id,
        )
        with self._lock:
            decision = self._start(request)
        if not decision.accepted:
            goal_handle.abort()
            return self._action_result(decision.result_code, decision.message, decision.execution_id)
        execution_id = decision.execution_id
        while rclpy.ok():
            with self._lock:
                if goal_handle.is_cancel_requested:
                    self._stop(execution_id, return_to_manual=True)
                snapshot = self._machine.snapshot
                result = self._action_results.get(execution_id)
                feedback = self._feedback(snapshot)
            goal_handle.publish_feedback(feedback)
            if result is not None:
                code, message = result
                if code == ResultCode.CANCELED:
                    goal_handle.canceled()
                elif code == ResultCode.SUCCEEDED:
                    goal_handle.succeed()
                else:
                    goal_handle.abort()
                return self._action_result(code, message, execution_id)
            await self._action_tick
        return self._action_result(ResultCode.INTERNAL_ERROR, "ROS shutdown", execution_id)

    def _on_start(self, request, response):
        with self._lock:
            decision = self._start(StartRequest(
                request.task_id, request.request_auto_mode, request.dry_run, request.request_id
            ))
        response.accepted = decision.accepted
        response.reason_code = self._start_reason_code(decision)
        response.message = decision.message
        response.execution_id = decision.execution_id
        return response

    def _on_stop(self, request, response):
        with self._lock:
            accepted, message = self._stop(
                request.execution_id, return_to_manual=request.return_to_manual
            )
        response.accepted = accepted
        response.message = message
        return response

    def _on_list(self, _request, response):
        response.tasks = [self._task_info(item) for item in self._registry.all()]
        return response

    def _on_status(self, _request, response):
        response.status = self._status_message()
        return response

    def _start(self, request: StartRequest):
        task = self._registry.get(request.task_id)
        available = bool(task and task.runnable)
        reason = "unknown task" if task is None else task.reason
        decision = self._machine.request_start(request, task_available=available, reason=reason)
        if not decision.accepted or decision.duplicate:
            return decision
        assert task is not None
        if request.request_auto_mode and not task.supports_auto_mode:
            return self._reject_started(decision.execution_id, ResultCode.REJECTED, "task does not support AUTO")
        if request.dry_run and not task.supports_dry_run:
            return self._reject_started(decision.execution_id, ResultCode.REJECTED, "task does not support dry run")
        active_profile = self._active_nav2_profile
        if (not request.dry_run and task.task_id != "return_home" and (
            task.nav2_profile != active_profile
            or bool(self.get_parameter("force_runtime_reconfigure").value)
        )):
            self._machine.transition(MissionState.CONFIGURING, execution_id=decision.execution_id,
                                     stage="nav2_profile", message=f"switching Nav2 to {task.nav2_profile}")
            self._pending_runtime_start = (decision, task, request)
            if not self._configure_runtime_client.server_is_ready():
                return self._reject_started(decision.execution_id, ResultCode.CONFIGURATION_FAILED,
                                            "runtime manager /system/configure is unavailable")
            goal = ConfigureSystem.Goal(parameter="nav2_profile", value=task.nav2_profile)
            self._configure_runtime_client.send_goal_async(goal).add_done_callback(self._on_runtime_goal)
            self._publish_status()
            return decision
        try:
            route = self._load_route(task)
        except RegistryError as exc:
            return self._reject_started(decision.execution_id, ResultCode.CONFIGURATION_FAILED, str(exc))
        if route.frame_id != task.frame_id:
            return self._reject_started(
                decision.execution_id, ResultCode.CONFIGURATION_FAILED,
                f"registry frame {task.frame_id} differs from route frame {route.frame_id}",
            )
        if route.projection_points() and not self.get_parameter("skip_coordinate_projection").value:
            if not self._from_ll_client.service_is_ready():
                return self._reject_started(
                    decision.execution_id, ResultCode.CONFIGURATION_FAILED,
                    "map projection service /fromLL is unavailable",
                )
            self._machine.transition(
                MissionState.CONFIGURING, execution_id=decision.execution_id,
                stage="project_waypoints", message="projecting latitude/longitude waypoint coordinates",
            )
            try:
                self._start_coordinate_projection(decision.execution_id, task, route, request)
            except ValueError as exc:
                return self._reject_started(decision.execution_id, ResultCode.CONFIGURATION_FAILED, str(exc))
            self._publish_status()
            return decision
        return self._configure_route(decision, task, route, request)

    def _on_runtime_goal(self, future) -> None:
        with self._lock:
            pending = self._pending_runtime_start
            if pending is None:
                return
            try:
                handle = future.result()
                if not handle.accepted:
                    raise RuntimeError("runtime manager rejected profile switch")
                handle.get_result_async().add_done_callback(self._on_runtime_result)
            except Exception as exc:
                self._pending_runtime_start = None
                self._reject_started(pending[0].execution_id, ResultCode.CONFIGURATION_FAILED, str(exc))

    def _on_runtime_result(self, future) -> None:
        with self._lock:
            pending = self._pending_runtime_start
            if pending is None:
                return
            self._pending_runtime_start = None
            try:
                result = future.result().result
                if not result.success:
                    raise RuntimeError(result.message)
                decision, task, request = pending
                self._active_nav2_profile = task.nav2_profile
                try:
                    route = self._load_route(task)
                except RegistryError as exc:
                    self._reject_started(decision.execution_id, ResultCode.CONFIGURATION_FAILED, str(exc))
                    return
                if route.projection_points() and not self.get_parameter("skip_coordinate_projection").value:
                    if not self._from_ll_client.service_is_ready():
                        self._reject_started(
                            decision.execution_id, ResultCode.CONFIGURATION_FAILED,
                            "map projection service /fromLL is unavailable",
                        )
                        return
                    try:
                        self._start_coordinate_projection(decision.execution_id, task, route, request)
                    except ValueError as exc:
                        self._reject_started(decision.execution_id, ResultCode.CONFIGURATION_FAILED, str(exc))
                        return
                    self._publish_status()
                    return
                self._configure_route(decision, task, route, request)
            except Exception as exc:
                self._reject_started(pending[0].execution_id, ResultCode.CONFIGURATION_FAILED, str(exc))

    def _on_runtime_status(self, message: Nav2RuntimeStatus) -> None:
        with self._lock:
            self._runtime_state = message.state
            self._active_nav2_profile = message.active_profile
            if message.state == Nav2RuntimeStatus.FAILED and self._machine.active:
                self._complete(self._machine.snapshot.execution_id, ResultCode.CONFIGURATION_FAILED, message.message)

    def _configure_route(self, decision, task: TaskDefinition, route: Route, request: StartRequest,
                         anchor: tuple[float, float] | None = None):
        """Set Task 3's projected strict goals before activating its route."""
        try:
            route = self._transform_route_for_cart_test(route, anchor)
        except ValueError as exc:
            return self._reject_started(decision.execution_id, ResultCode.CONFIGURATION_FAILED, str(exc))
        if task.nav2_profile != "task3" or request.dry_run or task.executor == "task4_composite":
            return self._finish_configure_route(decision, task, route, request)
        if not self._task3_goal_checker_client.service_is_ready():
            return self._reject_started(
                decision.execution_id, ResultCode.CONFIGURATION_FAILED,
                "Task 3 goal checker parameter service is unavailable",
            )
        strict = [
            waypoint for waypoint in route.waypoints
            if waypoint.waypoint_type in {"dock_approach", "dock"}
        ]
        parameter_request = SetParameters.Request()
        parameter_request.parameters = [
            Parameter(
                name="general_goal_checker.heading_required_goal_xs",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                    double_array_value=[waypoint.x for waypoint in strict],
                ),
            ),
            Parameter(
                name="general_goal_checker.heading_required_goal_ys",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                    double_array_value=[waypoint.y for waypoint in strict],
                ),
            ),
        ]
        self._pending_task3_goal_checker = (decision, task, route, request)
        self._task3_goal_checker_client.call_async(parameter_request).add_done_callback(
            self._finish_task3_goal_checker_configuration
        )
        return decision

    def _transform_route_for_cart_test(self, route: Route, anchor: tuple[float, float] | None) -> Route:
        if not self.get_parameter("waypoint_transform_enabled").value:
            return route
        if anchor is None:
            raise ValueError("waypoint transform requires a projected latitude/longitude anchor")
        return Route(
            route.frame_id,
            transform_waypoints(
                route.waypoints,
                anchor[0], anchor[1],
                float(self.get_parameter("waypoint_transform_rotation_rad").value),
                float(self.get_parameter("waypoint_transform_scale").value),
                replace,
            ),
            route.stages, route.full_sequence_stages, route.constraints,
        )

    def _finish_task3_goal_checker_configuration(self, future) -> None:
        with self._lock:
            pending = self._pending_task3_goal_checker
            self._pending_task3_goal_checker = None
            if pending is None:
                return
            decision, task, route, request = pending
            if not self._machine.is_current(decision.execution_id):
                return
            try:
                response = future.result()
                if not all(result.successful for result in response.results):
                    reason = next(result.reason for result in response.results if not result.successful)
                    raise RuntimeError(reason or "controller rejected Task 3 strict-goal parameters")
            except Exception as exc:
                self._reject_started(
                    decision.execution_id, ResultCode.CONFIGURATION_FAILED,
                    f"unable to configure Task 3 strict goals: {exc}",
                )
                return
            self._finish_configure_route(decision, task, route, request)

    def _finish_configure_route(self, decision, task: TaskDefinition, route: Route, request: StartRequest):
        """Finish setup after any YAML GPS coordinates are expressed in map."""
        if task.executor == "staged_docking":
            self._dock_corrections.clear()
            for waypoint in route.waypoints:
                if waypoint.waypoint_id in self._dock_nominal_pub:
                    self._publish_dock_nominal(waypoint)
        self._active_route = route
        self._active_task = task
        self._navigation.set_frame_id(route.frame_id)
        self._configure_task1_cardinal_walls(task, route)
        self._configure_task4_buoy_walls(task, route)
        # Task 1's only required runtime input is the route just validated
        # above.  Publish it before requesting AUTO so AUTO admission does not
        # depend on a Nav2 goal that has not been sent yet.
        self._activate_task_readiness(decision.execution_id, task)
        if self._machine.snapshot.state == MissionState.VALIDATING:
            self._machine.transition(
                MissionState.CONFIGURING, execution_id=decision.execution_id,
                stage="configure", message="task route validated",
            )
        else:
            self._machine.update_progress(
                decision.execution_id, "configure", 1.0, "task route validated"
            )
        if request.dry_run:
            self._navigation.publish_preview(route.waypoints)
            self._complete(decision.execution_id, ResultCode.SUCCEEDED, "dry run validated route and profile")
            return decision
        if request.request_auto_mode:
            # Start Nav2 while command selection remains zero/MANUAL.  Waiting
            # for a fresh Nav2 command before starting the executor would form a
            # deadlock with SafetySupervisor's freshness interlock.
            self._machine.transition(
                MissionState.WAITING_FOR_AUTO_PERMISSION,
                execution_id=decision.execution_id,
                stage="permission",
                message="waiting for AUTO permission before starting Nav2",
            )
            self._pending_start = (decision.execution_id, task)
            self._auto_permission_deadline_ns = (
                self.get_clock().now().nanoseconds
                + int(float(self.get_parameter("auto_permission_timeout_sec").value) * 1e9)
            )
            self._auto_mode_request_sent = False
            self._request_auto_mode()
        else:
            self._begin_executor(decision.execution_id, task)
        self._publish_status()
        return decision

    def _start_coordinate_projection(
        self, execution_id: str, task: TaskDefinition, route: Route, request: StartRequest
    ) -> None:
        points = route.projection_points()
        anchor = self._cart_anchor()
        if anchor is not None:
            points += (anchor,)
        self._pending_coordinate_projection = {
            "execution_id": execution_id,
            "task": task,
            "route": route,
            "request": request,
            "points": points,
            "route_point_count": len(route.waypoints),
            "projected": [],
            "next_index": 0,
            "attempts": 1,
            "request_serial": 0,
        }
        timeout_sec = max(0.0, float(
            self.get_parameter("coordinate_projection_timeout_sec").value
        ))
        self._coordinate_projection_deadline_ns = (
            self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        ) if timeout_sec else None
        self._request_next_coordinate_projection()

    def _cart_anchor(self) -> GeodeticPoint | None:
        if not self.get_parameter("waypoint_transform_enabled").value:
            return None
        latitude = float(self.get_parameter("waypoint_transform_anchor_latitude").value)
        longitude = float(self.get_parameter("waypoint_transform_anchor_longitude").value)
        if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
            raise ValueError("waypoint transform anchor latitude/longitude is out of range")
        return GeodeticPoint(latitude, longitude)

    def _request_next_coordinate_projection(self) -> None:
        """Serialize /fromLL requests; navsat_transform drops bursts of service calls."""
        pending = self._pending_coordinate_projection
        if pending is None:
            return
        if "retry_after_ns" in pending:
            return
        points = pending["points"]
        index = pending["next_index"]
        if index >= len(points):
            projected = tuple(pending["projected"])
            route_points = projected[:pending["route_point_count"]]
            if pending["route"].projection_is_degenerate(route_points):
                retry_sec = float(self.get_parameter("coordinate_projection_retry_sec").value)
                if retry_sec <= 0.0:
                    retry_sec = 0.1
                pending["projected"].clear()
                pending["next_index"] = 0
                pending["attempts"] += 1
                pending["retry_after_ns"] = self.get_clock().now().nanoseconds + int(retry_sec * 1e9)
                self._machine.update_progress(
                    pending["execution_id"], "wait_for_map_projection", 0.0,
                    "/fromLL returned a degenerate route; waiting for map projection initialization",
                )
                return
            self._pending_coordinate_projection = None
            self._coordinate_projection_deadline_ns = None
            resolved = pending["route"].with_projected_points(route_points)
            anchor = projected[-1] if len(projected) > len(route_points) else None
            decision = type("Decision", (), {"execution_id": pending["execution_id"]})()
            self._configure_route(decision, pending["task"], resolved, pending["request"], anchor)
            return
        point = points[index]
        projection_request = FromLL.Request()
        projection_request.ll_point = GeoPoint(
            latitude=point.latitude, longitude=point.longitude, altitude=point.altitude
        )
        # The Humble /fromLL service can lose an individual DDS response
        # while remaining available for the next request.  Associate every
        # request with a serial so a late reply from a timed-out attempt is
        # ignored rather than being appended as a duplicate waypoint.
        pending["request_serial"] += 1
        request_serial = pending["request_serial"]
        pending["request_started_ns"] = self.get_clock().now().nanoseconds
        future = self._from_ll_client.call_async(projection_request)
        future.add_done_callback(
            lambda result, serial=request_serial: self._finish_coordinate_projection(result, serial)
        )

    def _finish_coordinate_projection(self, future, request_serial: int) -> None:
        with self._lock:
            pending = self._pending_coordinate_projection
            if (
                pending is None
                or request_serial != pending["request_serial"]
                or not self._machine.is_current(pending["execution_id"])
            ):
                return
            try:
                response = future.result()
                pending["projected"].append((
                    float(response.map_point.x), float(response.map_point.y)
                ))
            except Exception as exc:
                execution_id = pending["execution_id"]
                self._pending_coordinate_projection = None
                self._coordinate_projection_deadline_ns = None
                self._complete(
                    execution_id, ResultCode.CONFIGURATION_FAILED,
                    f"map projection for waypoint coordinates failed: {exc}",
                )
                return
            pending.pop("request_started_ns", None)
            pending["next_index"] += 1
            self._machine.update_progress(
                pending["execution_id"], "project_waypoints",
                pending["next_index"] / len(pending["points"]),
                f"projected waypoint {pending['next_index']}/{len(pending['points'])}",
            )
            # Keep requests serial, but do not impose a per-waypoint delay.
            self._request_next_coordinate_projection()

    def _reject_started(self, execution_id: str, code: ResultCode, message: str):
        self._machine.finish(execution_id, code, message)
        self._action_results[execution_id] = (code, message)
        self._publish_status()
        self._machine.reset_to_idle(execution_id)
        self._active_task = None
        self._set_task2_enabled(False)
        self._set_task1_cardinal_walls(False)
        self._publish_task_readiness(False, False, "")
        self._publish_status()
        return type("Decision", (), {"accepted": False, "result_code": code, "message": message,
                                      "execution_id": execution_id, "duplicate": False})()

    def _load_route(self, task: TaskDefinition) -> Route:
        root = self._waypoint_share_path() if task.route_package == "waypoint_publisher" else self._registry_path().parent
        route = self._loader.load(root / task.route, task.route_key)
        if task.task_id != "task1":
            return route
        waypoints = tuple(
            waypoint for waypoint in route.waypoints if waypoint.competition_id != "1"
        )
        if not waypoints:
            raise RegistryError("Task1 route has no navigation waypoints after GPS1")
        return Route(
            route.frame_id, waypoints, route.stages,
            route.full_sequence_stages, route.constraints,
        )

    def _begin_executor(self, execution_id: str, task: TaskDefinition) -> None:
        if not self._machine.is_current(execution_id) or self._active_route is None:
            return
        self._pending_start = None
        self._pending_coordinate_projection = None
        self._machine.transition(MissionState.RUNNING, execution_id=execution_id,
                                 stage="starting", message="starting task executor")
        self._navigation.set_feedback_callback(
            self._on_task1_navigation_feedback if task.features.get("cardinal_walls") is True else None
        )
        if task.executor == "waypoint_sequence":
            executor = WaypointSequenceExecutor(self._navigation)
            executor.start(execution_id, self._active_route, self._feedback_update(execution_id),
                           self._executor_complete(execution_id))
        elif task.executor == "staged_docking":
            executor = StagedDockingExecutor(
                self._navigation, self._create_wait_timer, self.destroy_timer,
                self._dock_target_for_navigation, self._on_docking_stage_succeeded,
            )
            executor.start(execution_id, self._active_route, self._feedback_update(execution_id),
                           self._executor_complete(execution_id))
        elif task.executor == "task2_mppi":
            executor = Task2MppiExecutor(self._set_task2_enabled)
            executor.start(execution_id, self._active_route, self._feedback_update(execution_id),
                           self._executor_complete(execution_id))
        elif task.executor == "task4_composite":
            executor = Task4CompositeExecutor(
                self._navigation,
                lambda profile, complete: self._switch_task4_profile(execution_id, profile, complete),
                lambda goals, complete: self._configure_task4_strict_goals(execution_id, goals, complete),
                lambda policy, complete: self._configure_task4_docking_motion(execution_id, policy, complete),
            )
            executor.start(execution_id, self._active_route, self._feedback_update(execution_id),
                           self._executor_complete(execution_id))
        elif task.executor == "dummy":
            executor = DummyExecutor()
            executor.start(execution_id, self._feedback_update(execution_id),
                           self._executor_complete(execution_id))
        self._active_executor = executor

    def _on_dock_correction(self, berth: str, message: PoseStamped) -> None:
        if message.header.frame_id == "map":
            self._dock_corrections[berth] = message

    def _dock_target_for_navigation(self, waypoint: Waypoint) -> Waypoint:
        publisher = self._dock_nominal_pub.get(waypoint.waypoint_id)
        if publisher is None:
            return waypoint
        self._publish_dock_nominal(waypoint)
        correction = self._dock_corrections.get(waypoint.waypoint_id)
        if correction is None:
            return waypoint
        now = self.get_clock().now().nanoseconds
        stamp = correction.header.stamp
        age_ns = now - (stamp.sec * 1_000_000_000 + stamp.nanosec)
        dx, dy = correction.pose.position.x - waypoint.x, correction.pose.position.y - waypoint.y
        if age_ns < 0 or age_ns > 2_000_000_000 or dx * dx + dy * dy > 1.0 ** 2:
            return waypoint
        yaw = math.atan2(2.0 * correction.pose.orientation.w * correction.pose.orientation.z,
                         1.0 - 2.0 * correction.pose.orientation.z ** 2)
        if abs(math.atan2(math.sin(yaw - waypoint.yaw), math.cos(yaw - waypoint.yaw))) > 0.35:
            return waypoint
        self.get_logger().info(f"using LiDAR berth correction for {waypoint.waypoint_id}: dx={dx:.2f}m dy={dy:.2f}m")
        return replace(waypoint, x=correction.pose.position.x, y=correction.pose.position.y, yaw=yaw)

    def _publish_dock_nominal(self, waypoint: Waypoint) -> None:
        nominal = PoseStamped()
        nominal.header.frame_id = "map"
        nominal.header.stamp = self.get_clock().now().to_msg()
        nominal.pose.position.x, nominal.pose.position.y = waypoint.x, waypoint.y
        nominal.pose.orientation.z, nominal.pose.orientation.w = math.sin(waypoint.yaw / 2.0), math.cos(waypoint.yaw / 2.0)
        self._dock_nominal_pub[waypoint.waypoint_id].publish(nominal)

    def _switch_task4_profile(self, execution_id: str, profile: str, complete) -> None:
        """Switch Task 4 profiles while its executor owns the next Nav2 goal."""
        if not self._machine.is_current(execution_id):
            return
        if not self._configure_runtime_client.server_is_ready():
            complete(False, "runtime manager /system/configure is unavailable")
            return
        goal = ConfigureSystem.Goal(parameter="nav2_profile", value=profile)

        def sent(future) -> None:
            with self._lock:
                if not self._machine.is_current(execution_id):
                    return
                try:
                    handle = future.result()
                    if not handle.accepted:
                        raise RuntimeError("runtime manager rejected profile switch")
                    handle.get_result_async().add_done_callback(done)
                except Exception as exc:
                    complete(False, str(exc))

        def done(future) -> None:
            with self._lock:
                if not self._machine.is_current(execution_id):
                    return
                try:
                    result = future.result().result
                    if not result.success:
                        raise RuntimeError(result.message)
                    self._active_nav2_profile = profile
                    complete(True, result.message)
                except Exception as exc:
                    complete(False, str(exc))

        self._configure_runtime_client.send_goal_async(goal).add_done_callback(sent)

    def _configure_task4_strict_goals(self, execution_id: str, goals: tuple[Waypoint, ...], complete) -> None:
        """Apply Task 3 heading checks only to the Task 4 docking target in flight."""
        if not self._machine.is_current(execution_id):
            return
        if not self._task3_goal_checker_client.service_is_ready():
            complete(False, "Task 3 goal checker parameter service is unavailable")
            return
        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                name="general_goal_checker.heading_required_goal_xs",
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                                     double_array_value=[waypoint.x for waypoint in goals]),
            ),
            Parameter(
                name="general_goal_checker.heading_required_goal_ys",
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                                     double_array_value=[waypoint.y for waypoint in goals]),
            ),
        ]

        def done(future) -> None:
            with self._lock:
                if not self._machine.is_current(execution_id):
                    return
                try:
                    response = future.result()
                    if not all(result.successful for result in response.results):
                        reason = next(result.reason for result in response.results if not result.successful)
                        raise RuntimeError(reason or "controller rejected Task 3 strict-goal parameters")
                    complete(True, "")
                except Exception as exc:
                    complete(False, f"unable to configure Task 3 strict goals: {exc}")

        self._task3_goal_checker_client.call_async(request).add_done_callback(done)

    def _configure_task4_docking_motion(self, execution_id: str, policy: str, complete) -> None:
        if not self._machine.is_current(execution_id):
            return
        if not self._task3_goal_checker_client.service_is_ready():
            complete(False, "controller parameter service is unavailable")
            return
        limits = {
            "forward": (0.0, 0.5, 0.0, 0.0, 21, 1),
            "sway_negative": (0.0, 0.0, -0.5, 0.0, 1, 21),
            "omni": (-0.5, 0.5, -0.5, 0.5, 21, 21),
        }
        if policy not in limits:
            complete(False, f"unknown Task 4 docking motion policy: {policy}")
            return
        min_x, max_x, min_y, max_y, samples_x, samples_y = limits[policy]
        request = SetParameters.Request()
        request.parameters = [
            Parameter(name=f"FollowPath.{name}", value=ParameterValue(type=kind, **{field: value}))
            for name, kind, field, value in (
                ("min_vel_x", ParameterType.PARAMETER_DOUBLE, "double_value", min_x),
                ("max_vel_x", ParameterType.PARAMETER_DOUBLE, "double_value", max_x),
                ("min_vel_y", ParameterType.PARAMETER_DOUBLE, "double_value", min_y),
                ("max_vel_y", ParameterType.PARAMETER_DOUBLE, "double_value", max_y),
                ("vx_samples", ParameterType.PARAMETER_INTEGER, "integer_value", samples_x),
                ("vy_samples", ParameterType.PARAMETER_INTEGER, "integer_value", samples_y),
            )
        ]

        def done(future) -> None:
            try:
                response = future.result()
                if not all(result.successful for result in response.results):
                    reason = next(result.reason for result in response.results if not result.successful)
                    raise RuntimeError(reason or "controller rejected docking motion parameters")
                complete(True, "")
            except Exception as exc:
                complete(False, f"unable to configure Task 4 docking motion: {exc}")

        self._task3_goal_checker_client.call_async(request).add_done_callback(done)

    def _set_task2_enabled(self, enabled: bool) -> None:
        self._task2_enabled_pub.publish(Bool(data=enabled))

    def _on_task2_goal_reached(self, message: Bool) -> None:
        if not message.data:
            return
        with self._lock:
            if isinstance(self._active_executor, Task2MppiExecutor):
                self._active_executor.goal_reached()

    def _configure_task1_cardinal_walls(self, task: TaskDefinition, route: Route) -> None:
        """Prepare the Task1 GPS3 stage gate from the validated route."""
        self._set_task1_cardinal_walls(False)
        self._task1_wall_enable_after_remaining = None
        if task.features.get("cardinal_walls") is not True:
            return
        by_competition_id = {waypoint.competition_id: waypoint for waypoint in route.waypoints}
        gps3 = by_competition_id.get("3")
        gps4 = by_competition_id.get("4")
        if gps3 is None or gps4 is None:
            self.get_logger().error("Task1 route has no GPS3/GPS4; cardinal walls remain disabled")
            return
        dx, dy = gps4.x - gps3.x, gps4.y - gps3.y
        if dx * dx + dy * dy < 1.0e-8:
            self.get_logger().error("Task1 GPS3 and GPS4 coincide; cardinal walls remain disabled")
            return
        gps3_index = route.waypoints.index(gps3)
        self._task1_wall_enable_after_remaining = len(route.waypoints) - gps3_index
        self._task1_cardinal_heading_pub.publish(Float64(data=math.atan2(dy, dx)))

    def _on_task1_navigation_feedback(self, feedback) -> None:
        """Latch virtual walls only after Nav2 reports competition GPS3 passed."""
        threshold = self._task1_wall_enable_after_remaining
        if self._task1_walls_enabled or threshold is None:
            return
        if int(feedback.number_of_poses_remaining) < threshold:
            self._set_task1_cardinal_walls(True)
            self.get_logger().info("GPS3 reached: enabled cardinal virtual walls")

    def _set_task1_cardinal_walls(self, enabled: bool) -> None:
        self._task1_walls_enabled = enabled
        self._task1_cardinal_wall_enable_pub.publish(Bool(data=enabled))

    def _configure_task4_buoy_walls(self, task: TaskDefinition, route: Route) -> None:
        self._set_task4_buoy_walls(False)
        if task.task_id != "task4":
            return
        by_id = {waypoint.waypoint_id: waypoint for waypoint in route.waypoints}
        gps14, gps16 = by_id.get("14_outbound"), by_id.get("16")
        if gps14 is None or gps16 is None:
            self.get_logger().error("Task4 route has no GPS14/GPS16; buoy walls remain disabled")
            return
        dx, dy = gps16.x - gps14.x, gps16.y - gps14.y
        if dx * dx + dy * dy < 1.0e-8:
            self.get_logger().error("Task4 GPS14 and GPS16 coincide; buoy walls remain disabled")
            return
        self._task4_buoy_wall_heading_pub.publish(Float64(data=math.atan2(dy, dx)))

    def _on_docking_stage_succeeded(self, _stage: str, poses: Sequence[Waypoint]) -> None:
        if self._active_task is None or self._active_task.task_id != "task4" or not poses:
            return
        reached_id = poses[-1].waypoint_id
        if reached_id == "14_outbound":
            self._set_task4_buoy_walls(True)
        elif reached_id == "16":
            self._set_task4_buoy_walls(False)

    def _set_task4_buoy_walls(self, enabled: bool) -> None:
        self._task4_walls_enabled = enabled
        self._task4_buoy_wall_enable_pub.publish(Bool(data=enabled))

    def _activate_task_readiness(self, execution_id: str, task: TaskDefinition) -> None:
        if self._machine.is_current(execution_id):
            self._publish_task_readiness(
                True, self._requirements_ready(task), task.control_policy
            )

    def _requirements_ready(self, task: TaskDefinition) -> bool:
        return all(self._runtime_readiness[name] for name in required_runtime_readiness(task))

    def _request_auto_mode(self) -> None:
        """Request AUTO through the sole typed mode owner, never by topic spoofing."""
        if not self._set_mode_client.service_is_ready():
            return
        self._auto_mode_request_sent = True
        request = SetControlMode.Request()
        request.requested_mode = SetControlMode.Request.MODE_AUTO
        request.request_id = f"mission-{self._machine.snapshot.execution_id}"
        future = self._set_mode_client.call_async(request)

        def completed(result_future) -> None:
            try:
                result = result_future.result()
                if not result.accepted:
                    self.get_logger().error(f"AUTO mode request rejected: {result.message}")
            except Exception as exc:
                self.get_logger().error(f"AUTO mode request failed: {exc}")

        future.add_done_callback(completed)

    def _create_wait_timer(self, seconds: float, callback: Callable[[], None]):
        return self.create_timer(seconds if seconds > 0 else 0.001, callback, callback_group=self._cb_group)

    def _feedback_update(self, execution_id: str):
        def update(stage: str, progress: float, message: str) -> None:
            with self._lock:
                if self._machine.update_progress(execution_id, stage, progress, message):
                    self._publish_status()
        return update

    def _executor_complete(self, execution_id: str):
        def complete(result) -> None:
            with self._lock:
                mapping = {
                    ExecutorStatus.SUCCEEDED: ResultCode.SUCCEEDED,
                    ExecutorStatus.CANCELED: ResultCode.CANCELED,
                    ExecutorStatus.REJECTED: ResultCode.NAVIGATION_FAILED,
                    ExecutorStatus.NAVIGATION_FAILED: ResultCode.NAVIGATION_FAILED,
                    ExecutorStatus.INTERNAL_ERROR: ResultCode.INTERNAL_ERROR,
                }
                self._complete(execution_id, mapping[result.status], result.message)
        return complete

    def _stop(self, execution_id: str, *, return_to_manual: bool = False) -> tuple[bool, str]:
        if not self._machine.is_current(execution_id) or not self._machine.active:
            return False, "execution is not active"
        was_canceling = self._machine.snapshot.state == MissionState.CANCELING
        self._machine.begin_cancel(execution_id)
        self._set_task2_enabled(False)
        self._set_task1_cardinal_walls(False)
        self._set_task4_buoy_walls(False)
        self._publish_task_readiness(False, False, "")
        self._pending_start = None
        self._pending_coordinate_projection = None
        self._coordinate_projection_deadline_ns = None
        self._auto_permission_deadline_ns = None
        self._auto_mode_request_sent = False
        if self._active_executor and not was_canceling:
            self._active_executor.cancel(execution_id)
        elif not self._active_executor and not was_canceling:
            self._complete(execution_id, ResultCode.CANCELED, "task canceled before navigation started")
        if return_to_manual:
            self._request_manual_mode(execution_id)
        self._publish_status()
        return True, "cancellation requested"

    def _request_manual_mode(self, execution_id: str) -> None:
        if execution_id in self._manual_mode_requested or not self._set_mode_client.service_is_ready():
            return
        request = SetControlMode.Request()
        request.requested_mode = SetControlMode.Request.MODE_MANUAL
        request.request_id = f"{execution_id}:stop-manual"
        self._manual_mode_requested.add(execution_id)
        future = self._set_mode_client.call_async(request)
        future.add_done_callback(
            lambda result: self.get_logger().error(
                f"Mode Manager MANUAL request failed: {result.exception()}"
            ) if result.exception() else None
        )

    def _complete(self, execution_id: str, code: ResultCode, message: str) -> None:
        if not self._machine.finish(execution_id, code, message):
            return
        self._action_results[execution_id] = (code, message)
        self._pending_start = None
        self._pending_coordinate_projection = None
        self._coordinate_projection_deadline_ns = None
        self._auto_permission_deadline_ns = None
        self._auto_mode_request_sent = False
        self._active_executor = None
        self._active_task = None
        self._set_task2_enabled(False)
        self._set_task1_cardinal_walls(False)
        self._set_task4_buoy_walls(False)
        self._navigation.set_feedback_callback(None)
        self._publish_status()
        self._publish_task_readiness(False, False, "")
        self._machine.reset_to_idle(execution_id)
        self._publish_status()

    def _on_control_state(self, message: ControlState) -> None:
        with self._lock:
            self._auto_permitted = message.auto_permitted
            self._requested_mode = message.requested_mode
            self._effective_source = message.effective_source
            self._inhibit_reasons = list(message.inhibit_reasons)

    def _on_failsafe_return(self, _message: Empty) -> None:
        """Preempt one active task, then start the home route through the normal owner."""
        with self._lock:
            if self._active_task is not None and self._active_task.task_id == "return_home":
                return
            self._pending_failsafe_return = True
            snapshot = self._machine.snapshot
            if self._machine.active:
                self._stop(snapshot.execution_id, return_to_manual=False)

    def _on_runtime_readiness(self, name: str, message: Bool) -> None:
        with self._lock:
            self._runtime_readiness[name] = message.data
            task = self._active_task
            if task is not None and self._machine.active:
                self._publish_task_readiness(
                    True, self._requirements_ready(task), task.control_policy
                )

    def _tick(self) -> None:
        with self._lock:
            self._action_tick.set_result(None)
            self._action_tick = Future()
            snapshot = self._machine.snapshot
            pending_projection = self._pending_coordinate_projection
            if (
                pending_projection is not None
                and self._coordinate_projection_deadline_ns is not None
                and self.get_clock().now().nanoseconds >= self._coordinate_projection_deadline_ns
            ):
                execution_id = pending_projection["execution_id"]
                self._pending_coordinate_projection = None
                self._coordinate_projection_deadline_ns = None
                self._complete(
                    execution_id,
                    ResultCode.CONFIGURATION_FAILED,
                    "map projection service /fromLL timed out after "
                    f"{float(self.get_parameter('coordinate_projection_timeout_sec').value):g} seconds",
                )
                snapshot = self._machine.snapshot
            elif (
                pending_projection is not None
                and "request_started_ns" in pending_projection
                and self.get_clock().now().nanoseconds - pending_projection["request_started_ns"]
                >= int(max(0.1, float(
                    self.get_parameter("coordinate_projection_request_timeout_sec").value
                )) * 1e9)
            ):
                self._machine.update_progress(
                    pending_projection["execution_id"], "project_waypoints",
                    pending_projection["next_index"] / len(pending_projection["points"]),
                    f"map projection request {pending_projection['next_index'] + 1}/"
                    f"{len(pending_projection['points'])} timed out; retrying",
                )
                self._request_next_coordinate_projection()
            elif (
                pending_projection is not None
                and "retry_after_ns" in pending_projection
                and self.get_clock().now().nanoseconds >= pending_projection["retry_after_ns"]
            ):
                del pending_projection["retry_after_ns"]
                self._request_next_coordinate_projection()
            if snapshot.state == MissionState.WAITING_FOR_AUTO_PERMISSION:
                if not self._auto_mode_request_sent:
                    self._request_auto_mode()
                if self._auto_permitted:
                    pending = self._pending_start
                    self._auto_permission_deadline_ns = None
                    if pending is not None:
                        self._begin_executor(*pending)
                elif (
                    self._auto_permission_deadline_ns is not None
                    and self.get_clock().now().nanoseconds >= self._auto_permission_deadline_ns
                ):
                    if self._active_executor:
                        self._active_executor.cleanup(snapshot.execution_id)
                    self._complete(
                        snapshot.execution_id,
                        ResultCode.SAFETY_INHIBITED,
                        "AUTO permission timed out: " + "; ".join(self._inhibit_reasons),
                    )
            if self._pending_failsafe_return and not self._machine.active:
                self._pending_failsafe_return = False
                self._failsafe_return_count += 1
                decision = self._start(StartRequest(
                    "return_home", request_auto_mode=True, dry_run=False,
                    request_id=f"failsafe-return-home-{self._failsafe_return_count}",
                ))
                if not decision.accepted:
                    self.get_logger().error(f"return-home start rejected: {decision.message}")
            self._publish_status()

    def _publish_status(self) -> None:
        if self._machine.snapshot != self._last_snapshot:
            self._last_transition = self.get_clock().now().to_msg()
            self._last_snapshot = self._machine.snapshot
        self._status_pub.publish(self._status_message())

    def _publish_task_readiness(
        self, task_ready: bool, requirements_ready: bool, control_policy: str
    ) -> None:
        """Publish the mission-owned safety inputs with latched safe defaults."""
        self._task_ready = task_ready
        self._task_requirements_ready = requirements_ready
        self._active_control_policy = control_policy
        task_message = Bool()
        task_message.data = task_ready
        requirements_message = Bool()
        requirements_message.data = requirements_ready
        policy_message = String()
        policy_message.data = control_policy
        self._task_ready_pub.publish(task_message)
        self._task_requirements_ready_pub.publish(requirements_message)
        self._active_policy_pub.publish(policy_message)

    def _status_message(self) -> MissionStatus:
        snapshot = self._machine.snapshot
        message = MissionStatus()
        message.stamp = self.get_clock().now().to_msg()
        message.state = int(snapshot.state)
        message.task_id = snapshot.task_id
        message.execution_id = snapshot.execution_id
        message.stage = snapshot.stage
        message.progress = snapshot.progress
        message.message = snapshot.message
        message.requested_control_mode = self._requested_mode
        message.effective_control_source = self._effective_source
        message.auto_permitted = self._auto_permitted
        message.inhibit_reasons = self._inhibit_reasons
        message.last_transition_time = self._last_transition
        task = self._active_task or self._registry.get(snapshot.task_id)
        message.required_nav2_profile = task.nav2_profile if task else ""
        message.active_nav2_profile = self._active_nav2_profile
        message.runtime_state = self._runtime_state
        return message

    def _feedback(self, snapshot) -> RunTask.Feedback:
        message = RunTask.Feedback()
        message.state = int(snapshot.state)
        message.task_id = snapshot.task_id
        message.execution_id = snapshot.execution_id
        message.stage = snapshot.stage
        message.progress = snapshot.progress
        message.message = snapshot.message
        message.inhibit_reasons = self._inhibit_reasons
        return message

    def _action_result(self, code: ResultCode, message: str, execution_id: str) -> RunTask.Result:
        result = RunTask.Result()
        result.result_code = int(code)
        result.message = message
        result.execution_id = execution_id
        result.final_inhibit_reasons = self._inhibit_reasons
        return result

    @staticmethod
    def _task_info(task: TaskDefinition) -> TaskInfo:
        message = TaskInfo()
        availability = {
            "available": TaskInfo.AVAILABLE,
            "experimental": TaskInfo.EXPERIMENTAL,
            "not_implemented": TaskInfo.NOT_IMPLEMENTED,
            "disabled": TaskInfo.DISABLED,
        }
        message.task_id = task.task_id
        message.display_name = task.display_name
        message.availability = availability[task.availability]
        message.reason = task.reason
        message.supports_dry_run = task.supports_dry_run
        message.supports_auto_mode = task.supports_auto_mode
        return message

    @staticmethod
    def _start_reason_code(decision) -> int:
        if decision.accepted:
            return StartTask.Response.DUPLICATE_REQUEST if decision.duplicate else StartTask.Response.ACCEPTED
        if decision.result_code == ResultCode.CONFIGURATION_FAILED:
            return StartTask.Response.CONFIGURATION_FAILED
        return StartTask.Response.TASK_UNAVAILABLE


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = MissionManager()
        rclpy.spin(node)
    except RegistryError as exc:
        rclpy.logging.get_logger("mission_manager").fatal(f"mission registry invalid: {exc}")
        raise
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
