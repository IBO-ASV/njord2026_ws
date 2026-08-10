from types import SimpleNamespace

from mission_manager.operator_dispatcher_node import OperatorDispatcher
from njord_interfaces.action import RunTask
from njord_interfaces.msg import OperatorResponse


class _Future:
    def __init__(self, result=None):
        self._result = result
        self.callback = None

    def result(self):
        return self._result

    def add_done_callback(self, callback):
        self.callback = callback


def _dispatcher():
    dispatcher = object.__new__(OperatorDispatcher)
    dispatcher.responses = []
    dispatcher._respond = lambda command, code, message: dispatcher.responses.append((command, code, message))
    return dispatcher


def test_task_check_uses_dry_run_action_and_waits_for_its_result():
    dispatcher = _dispatcher()
    future = _Future()
    dispatcher._run = SimpleNamespace(
        server_is_ready=lambda: True,
        send_goal_async=lambda goal: setattr(dispatcher, "goal", goal) or future,
    )
    command = SimpleNamespace(task_id="task1", request_id=7)

    dispatcher._check(command)

    assert dispatcher.goal.task_id == "task1"
    assert dispatcher.goal.dry_run is True
    assert future.callback is not None
    assert dispatcher.responses == []


def test_task_check_reports_the_completed_action_result():
    dispatcher = _dispatcher()
    command = SimpleNamespace()
    completed = _Future(SimpleNamespace(result=SimpleNamespace(
        result_code=RunTask.Result.SUCCEEDED, message="dry run validated route and profile"
    )))

    dispatcher._check_result(command, completed)

    assert dispatcher.responses == [
        (command, OperatorResponse.OK, "dry run validated route and profile")
    ]
