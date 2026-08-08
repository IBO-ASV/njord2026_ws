from mission_manager.executors import (
    ExecutorStatus,
    StagedDockingExecutor,
    Task2MppiExecutor,
    WaypointSequenceExecutor,
)
from mission_manager.waypoint_config import Route, Waypoint


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
    _, accepted, completed = nav.sent[0]
    accepted(True)
    completed(ExecutorStatus.SUCCEEDED, "ok")
    assert results[0].status == ExecutorStatus.SUCCEEDED


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


def test_task2_mppi_withdraws_its_gate_before_reporting_canceled():
    enabled = []
    results = []
    executor = Task2MppiExecutor(enabled.append)
    executor.start("execution", _route(), lambda *_: None, results.append)
    assert enabled == [True]

    executor.cancel("execution")

    assert enabled == [True, False]
    assert results[0].status == ExecutorStatus.CANCELED
