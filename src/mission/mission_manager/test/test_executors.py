from mission_manager.executors import (
    ExecutorStatus, StagedDockingExecutor, Task2MppiExecutor, Task4CompositeExecutor,
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


def test_task4_composite_sends_the_required_route_and_profile_switches():
    nav = FakeNavigation()
    switches, strict, results = [], [], []
    callbacks = {"switch": [], "strict": []}
    route = Route("odom", tuple(
        Waypoint(waypoint_id, 0.0, 0.0, 0.0, waypoint_id, "dock")
        for waypoint_id in ("13", "14", "normal_dock", "normal_exit", "15", "16", "17", "parallel_dock")
    ), {}, {}, {})
    executor = Task4CompositeExecutor(
        nav,
        lambda profile, complete: (switches.append(profile), callbacks["switch"].append(complete)),
        lambda goals, complete: (strict.append(goals[0].waypoint_id), callbacks["strict"].append(complete)),
    )
    executor.start("execution", route, lambda *_: None, results.append)

    expected = ["13", "14", "normal_dock", "normal_exit", "15", "16", "17", "17", "parallel_dock"]
    for waypoint_id in expected:
        if callbacks["strict"]:
            callbacks["strict"].pop(0)(True, "")
        poses, accepted, completed = nav.sent[-1]
        assert [pose.waypoint_id for pose in poses] == [waypoint_id]
        accepted(True)
        completed(ExecutorStatus.SUCCEEDED, "ok")
        if waypoint_id == "14" and len(nav.sent) == 4:
            callbacks["switch"].pop(0)(True, "")
        if waypoint_id == "17" and len(nav.sent) == 7:
            callbacks["switch"].pop(0)(True, "")
    assert switches == ["task1", "task3"]
    assert strict == ["14", "normal_dock", "17", "parallel_dock"]
    assert results[0].status == ExecutorStatus.SUCCEEDED


def test_task4_composite_stops_after_a_rejected_profile_switch():
    nav = FakeNavigation()
    callbacks, results = [], []
    route = Route("odom", tuple(
        Waypoint(waypoint_id, 0.0, 0.0, 0.0, waypoint_id, "dock")
        for waypoint_id in ("13", "14", "normal_dock", "normal_exit", "15", "16", "17", "parallel_dock")
    ), {}, {}, {})
    executor = Task4CompositeExecutor(
        nav, lambda _profile, complete: callbacks.append(complete), lambda _goals, complete: complete(True, "")
    )
    executor.start("execution", route, lambda *_: None, results.append)
    for _ in range(4):
        nav.sent[-1][2](ExecutorStatus.SUCCEEDED, "ok")
    callbacks[0](False, "switch rejected")
    assert [poses[0].waypoint_id for poses, *_ in nav.sent] == ["13", "14", "normal_dock", "normal_exit"]
    assert results[0].status == ExecutorStatus.INTERNAL_ERROR


def test_task4_composite_cancels_during_profile_switch_and_ignores_late_result():
    nav = FakeNavigation()
    callbacks, results = [], []
    route = Route("odom", tuple(
        Waypoint(waypoint_id, 0.0, 0.0, 0.0, waypoint_id, "dock")
        for waypoint_id in ("13", "14", "normal_dock", "normal_exit", "15", "16", "17", "parallel_dock")
    ), {}, {}, {})
    executor = Task4CompositeExecutor(
        nav, lambda _profile, complete: callbacks.append(complete), lambda _goals, complete: complete(True, "")
    )
    executor.start("execution", route, lambda *_: None, results.append)
    for _ in range(4):
        nav.sent[-1][2](ExecutorStatus.SUCCEEDED, "ok")
    executor.cancel("execution")
    callbacks[0](True, "late")
    assert results[0].status == ExecutorStatus.CANCELED
    assert [poses[0].waypoint_id for poses, *_ in nav.sent] == ["13", "14", "normal_dock", "normal_exit"]


def test_task4_composite_stops_after_navigation_failure():
    nav = FakeNavigation()
    results = []
    route = Route("odom", tuple(
        Waypoint(waypoint_id, 0.0, 0.0, 0.0, waypoint_id, "dock")
        for waypoint_id in ("13", "14", "normal_dock", "normal_exit", "15", "16", "17", "parallel_dock")
    ), {}, {}, {})
    executor = Task4CompositeExecutor(
        nav, lambda _profile, complete: complete(True, ""), lambda _goals, complete: complete(True, "")
    )
    executor.start("execution", route, lambda *_: None, results.append)
    nav.sent[0][2](ExecutorStatus.NAVIGATION_FAILED, "blocked")
    assert len(nav.sent) == 1
    assert results[0].status == ExecutorStatus.NAVIGATION_FAILED
