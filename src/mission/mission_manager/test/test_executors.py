from mission_manager.executors import (
    ExecutorStatus, StagedDockingExecutor, Task2MppiExecutor,
    WaypointSequenceExecutor,
)
from mission_manager.mission_manager_node import _RosNavigationClient
from mission_manager.waypoint_config import Route, Waypoint
from geometry_msgs.msg import PoseStamped


class _MarkerPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _MarkerNode:
    class _Clock:
        class _Now:
            @staticmethod
            def to_msg():
                return PoseStamped().header.stamp

        @staticmethod
        def now():
            return _MarkerNode._Clock._Now()

    @staticmethod
    def get_clock():
        return _MarkerNode._Clock()


class FakeNavigation:
    def __init__(self):
        self.sent = []
        self.cancelled = False

    def send(self, poses, accepted, completed):
        self.sent.append((list(poses), accepted, completed))

    def cancel(self):
        self.cancelled = True


def _route():
    waypoints = (
        Waypoint("gate", 0.0, 0.0, 0.0, "gate", "gate"),
        Waypoint("dock", 1.0, 0.0, 0.0, "dock", "dock"),
        Waypoint("exit", 2.0, 0.0, 0.0, "exit", "goal"),
    )
    return Route("map", waypoints, {"stage_1_gate": ("gate",), "stage_1": ("dock",), "stage_2": ("exit",)}, {},
                 {"attempts": 2, "wait_time_s": 1})


def test_sequence_maps_nav_success_to_result():
    nav = FakeNavigation()
    results = []
    executor = WaypointSequenceExecutor(nav)
    executor.start("execution", _route(), lambda *_: None, results.append)
    for expected_id in ("gate", "dock", "exit"):
        poses, accepted, completed = nav.sent[-1]
        assert [pose.waypoint_id for pose in poses] == [expected_id]
        accepted(True)
        completed(ExecutorStatus.SUCCEEDED, "ok")
    assert results[0].status == ExecutorStatus.SUCCEEDED


def test_navigation_client_publishes_preview_markers_and_path():
    client = object.__new__(_RosNavigationClient)
    client._node = _MarkerNode()
    client._frame_id = "map"
    client._marker_publisher = _MarkerPublisher()
    client._path_publisher = _MarkerPublisher()
    waypoints = [
        Waypoint("first", 0.0, 0.0, 0.0, "first", "waypoint"),
        Waypoint("second", 2.0, 0.0, 0.0, "second", "waypoint"),
    ]

    client.publish_preview(waypoints)

    route, points = client._marker_publisher.messages[0].markers
    assert route.type == route.LINE_STRIP
    assert [(point.x, point.y) for point in points.points] == [(0.0, 0.0), (2.0, 0.0)]
    assert [pose.pose.position.x for pose in client._path_publisher.messages[0].poses] == [0.0, 2.0]


def test_staged_docking_cancels_wait_timer_and_ignores_late_callback():
    nav = FakeNavigation()
    timers = []
    canceled = []
    results = []
    executor = StagedDockingExecutor(nav, lambda _seconds, callback: timers.append(callback) or callback,
                                     canceled.append)
    executor.start("execution", _route(), lambda *_: None, results.append)
    _, accepted, completed = nav.sent[0]
    accepted(True)
    completed(ExecutorStatus.SUCCEEDED, "gate")
    _, accepted, completed = nav.sent[1]
    accepted(True)
    completed(ExecutorStatus.SUCCEEDED, "dock")
    executor.cancel("execution")
    assert nav.cancelled
    assert canceled
    timers[0]()
    assert results[0].status == ExecutorStatus.CANCELED


def test_staged_docking_does_not_wait_after_an_undocking_stage():
    nav = FakeNavigation()
    timers = []
    results = []
    executor = StagedDockingExecutor(nav, lambda _seconds, callback: timers.append(callback) or callback,
                                     lambda _timer: None)
    executor.start("execution", _route(), lambda *_: None, results.append)

    # Gate then berth.  The berth wait advances to the exit stage.
    nav.sent[0][2](ExecutorStatus.SUCCEEDED, "gate")
    nav.sent[1][2](ExecutorStatus.SUCCEEDED, "dock")
    timers[0]()
    assert len(nav.sent) == 3

    # stage_2 ends at an exit waypoint, so it completes immediately.
    nav.sent[2][2](ExecutorStatus.SUCCEEDED, "exit")
    assert results[0].status == ExecutorStatus.SUCCEEDED


def test_staged_docking_honors_explicit_stage_order_for_task4_style_route():
    nav = FakeNavigation()
    results = []
    route = Route(
        "map",
        (
            Waypoint("start", 0.0, 0.0, 0.0, "start", "start"),
            Waypoint("berth", 1.0, 0.0, 0.0, "berth", "dock"),
            Waypoint("exit", 2.0, 0.0, 0.0, "exit", "dock_approach"),
        ),
        {"first": ("start",), "dock": ("berth",), "leave": ("exit",)},
        {},
        {"stage_order": ["first", "dock", "leave"], "wait_time_s": 0},
    )
    executor = StagedDockingExecutor(
        nav, lambda _seconds, callback: (callback(), callback)[1], lambda _timer: None
    )
    executor.start("task4", route, lambda *_: None, results.append)
    for expected_id in ("start", "berth", "exit"):
        poses, _accepted, completed = nav.sent[-1]
        assert [pose.waypoint_id for pose in poses] == [expected_id]
        completed(ExecutorStatus.SUCCEEDED, "ok")
    assert results[0].status == ExecutorStatus.SUCCEEDED


def test_staged_docking_reports_only_nav2_confirmed_stage_successes():
    nav = FakeNavigation()
    reached = []
    route = Route(
        "map",
        (
            Waypoint("14_outbound", 0.0, 0.0, 0.0, "GPS 14", "waypoint"),
            Waypoint("16", 1.0, 0.0, 0.0, "GPS 16", "waypoint"),
        ),
        {"first": ("14_outbound",), "second": ("16",)},
        {},
        {"stage_order": ["first", "second"]},
    )
    executor = StagedDockingExecutor(
        nav, lambda _seconds, callback: callback, lambda _timer: None,
        stage_succeeded=lambda stage, poses: reached.append((stage, poses[-1].waypoint_id)),
    )
    executor.start("task4", route, lambda *_: None, lambda _result: None)
    nav.sent[0][2](ExecutorStatus.SUCCEEDED, "GPS14 reached")
    nav.sent[1][2](ExecutorStatus.SUCCEEDED, "GPS16 reached")
    assert reached == [("first", "14_outbound"), ("second", "16")]


def test_task2_mppi_withdraws_its_gate_before_reporting_canceled():
    enabled = []
    results = []
    executor = Task2MppiExecutor(enabled.append)
    executor.start("execution", _route(), lambda *_: None, results.append)
    executor.cancel("execution")
    assert enabled == [True, False]
    assert results[0].status == ExecutorStatus.CANCELED


def test_task2_mppi_reports_follow_path_goal_success_once():
    enabled = []
    results = []
    executor = Task2MppiExecutor(enabled.append)
    executor.start("execution", _route(), lambda *_: None, results.append)
    executor.goal_reached()
    executor.goal_reached()  # Delayed duplicate status must be harmless.
    assert enabled == [True, False]
    assert [result.status for result in results] == [ExecutorStatus.SUCCEEDED]


def test_dwell_requires_continuous_stationarity_and_cancels_poll_timer():
    nav=FakeNavigation(); timers=[]; canceled=[]; clock=[0.]; stationary=[False]
    executor=StagedDockingExecutor(nav,lambda _seconds,callback:timers.append(callback) or callback,
       canceled.append,stationary_gate=lambda _w:stationary[0],now=lambda:clock[0])
    results=[]; executor.start('dwell',_route(),lambda *_:None,results.append)
    nav.sent[0][2](ExecutorStatus.SUCCEEDED,'gate')
    nav.sent[1][2](ExecutorStatus.SUCCEEDED,'dock')
    poll=timers[0]
    clock[0]=10.;poll(); assert len(nav.sent)==2
    stationary[0]=True;poll()
    clock[0]=12.;poll(); assert len(nav.sent)==2  # 2 s settling + 1 s dwell
    stationary[0]=False;poll()
    stationary[0]=True;clock[0]=13.;poll()
    clock[0]=16.1;poll();assert len(nav.sent)==3
    assert canceled==[poll]
    # A previously queued periodic callback cannot advance the next stage.
    poll();assert len(nav.sent)==3


def test_cancel_while_waiting_for_stationary_input_finishes_once():
    nav=FakeNavigation();timers=[];clock=[0.];results=[]
    executor=StagedDockingExecutor(nav,lambda _seconds,callback:timers.append(callback) or callback,
       lambda _:None,stationary_gate=lambda _:False,now=lambda:clock[0])
    executor.start('cancel',_route(),lambda *_:None,results.append)
    nav.sent[0][2](ExecutorStatus.SUCCEEDED,'gate');nav.sent[1][2](ExecutorStatus.SUCCEEDED,'dock')
    executor.cancel('cancel');timers[0]()
    assert [r.status for r in results]==[ExecutorStatus.CANCELED]


def test_service_stop_cannot_request_an_invalid_ros_action_transition():
    import asyncio
    import threading
    from mission_manager.mission_manager_node import MissionManager
    from mission_manager.state_machine import ResultCode, StartDecision
    from njord_interfaces.action import RunTask
    class Handle:
        request=RunTask.Goal(task_id='task1',request_id='stop-test')
        is_cancel_requested=False  # StopTask service, not action cancel.
        def publish_feedback(self,_): pass
        def abort(self): self.terminal='abort'
        def canceled(self): raise AssertionError('EXECUTING -> CANCELED is invalid')
    manager=object.__new__(MissionManager)
    manager._lock=threading.RLock()
    manager._start=lambda _:StartDecision(True,ResultCode.SUCCEEDED,'accepted','execution')
    manager._action_results={'execution':(ResultCode.CANCELED,'stopped by service')}
    class Machine:
        snapshot=None
    manager._machine=Machine()
    manager._feedback=lambda _:RunTask.Feedback()
    manager._inhibit_reasons=[]
    handle=Handle()
    # Execute callback's loop only requires a valid context, no graph activity.
    import rclpy
    owned=not rclpy.ok()
    if owned:rclpy.init()
    try:
        result=asyncio.run(manager._execute_action(handle))
        assert result.result_code==ResultCode.CANCELED
        assert handle.terminal=='abort'
    finally:
        if owned:rclpy.shutdown()


def test_se2_docking_routes_only_configured_stages_and_keeps_berth_wait():
    from dataclasses import replace

    class Navigation(FakeNavigation):
        def send_docking(self, target, accepted, completed):
            self.sent.append(([target], accepted, completed))
            self.se2.append(target.waypoint_id)

    nav = Navigation()
    nav.se2 = []
    timers, results = [], []
    route = replace(_route(), constraints={
        "attempts": 2, "wait_time_s": 1,
        "se2_docking_stages": ["stage_1", "stage_2"],
    })
    corrected = replace(route.waypoints[1], x=1.1, yaw=0.2)
    executor = StagedDockingExecutor(
        nav, lambda _seconds, cb: timers.append(cb) or cb, lambda _: None,
        dock_target_correction=lambda _: corrected)
    executor.start("execution", route, lambda *_: None, results.append)
    assert nav.se2 == []
    nav.sent[-1][2](ExecutorStatus.SUCCEEDED, "gate")
    assert nav.se2 == ["dock"]
    assert nav.sent[-1][0][0] == corrected
    # Rejection retries the same SE2 stage, not normal navigation.
    nav.sent[-1][1](False)
    assert nav.se2 == ["dock", "dock"]
    nav.sent[-1][2](ExecutorStatus.SUCCEEDED, "dock")
    assert len(timers) == 1
    assert nav.se2 == ["dock", "dock"]
    timers[0]()
    assert nav.se2[-1] == "exit"
    executor.cancel("execution")
    assert nav.cancelled
    nav.sent[-1][2](ExecutorStatus.CANCELED, "cancel")
    assert results[-1].status == ExecutorStatus.CANCELED


def test_follow_path_adapter_keeps_bow_reference_and_controller_selection():
    from nav2_msgs.action import FollowPath

    client = object.__new__(_RosNavigationClient)
    start, target = _route().waypoints[:2]
    client._node = _MarkerNode()
    client._node._current_docking_pose = lambda _: start
    client._frame_id = "odom"
    client._path_publisher = _MarkerPublisher()
    client._marker_publisher = _MarkerPublisher()
    client._follow_client = object()
    sent = []
    client._send_goal = lambda *args: sent.append(args)
    client.send_docking(target, lambda _: None, lambda *_: None)
    server, goal, *_ = sent[0]
    assert server is client._follow_client
    assert isinstance(goal, FollowPath.Goal)
    assert goal.controller_id == "DockingSE2"
    assert goal.goal_checker_id == "docking_goal_checker"
    assert goal.path.header.frame_id == "odom"
    assert len(goal.path.poses) > 2
    assert goal.path.poses[0].pose.position.x == start.x
    assert goal.path.poses[-1].pose.position.x == target.x
    client._node._current_docking_pose = lambda _: None
    accepted = []
    client.send_docking(target, accepted.append, lambda *_: None)
    assert accepted == [False]
    assert len(sent) == 1


def test_cancel_before_acceptance_cancels_follow_path_and_stale_result_is_ignored():
    from concurrent.futures import Future
    from types import SimpleNamespace
    from action_msgs.msg import GoalStatus

    client = object.__new__(_RosNavigationClient)
    client._goal_handle = None
    client._callbacks = (lambda _: None, lambda *_: None)
    client.cancel()
    canceled, completions = [], []
    result = Future()
    handle = SimpleNamespace(accepted=True, cancel_goal_async=lambda: canceled.append(True),
                             get_result_async=lambda: result)
    response = Future()
    response.set_result(handle)
    client._on_goal_response(response, client._callbacks)
    assert canceled == [True]
    # Simulate a new dispatch before the old result arrives.
    client._callbacks = (lambda _: None, lambda *args: completions.append(args))
    result.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    assert completions == []
