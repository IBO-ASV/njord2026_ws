"""Mission-owned enable gate for the resident Task 2 MPPI stack."""

from __future__ import annotations

from .base import CompletionCallback, ExecutionResult, ExecutorStatus, FeedbackCallback
from ..waypoint_config import Route


class Task2MppiExecutor:
    """Enable Task 2's FollowPath client only for an active Mission execution.

    The MPPI/perception processes are resident when the Task 2 Nav2 profile is
    selected at bringup.  This executor is their sole Mission-level enable
    authority; cancelling a mission withdraws that enable before completing the
    Mission request.  The final command path remains Nav2 -> collision monitor
    -> Command Arbiter.
    """

    def __init__(self, set_enabled) -> None:
        self._set_enabled = set_enabled
        self._execution_id = ""
        self._complete: CompletionCallback | None = None
        self._finished = False

    def start(
        self, execution_id: str, route: Route, feedback: FeedbackCallback, complete: CompletionCallback
    ) -> None:
        self._execution_id = execution_id
        self._complete = complete
        self._finished = False
        if not route.waypoints:
            self._finish(ExecutorStatus.INTERNAL_ERROR, "Task 2 route has no waypoints")
            return
        self._set_enabled(True)
        feedback("navigate", 0.0, "Task 2 MPPI FollowPath gate enabled")

    def cancel(self, execution_id: str) -> None:
        if execution_id != self._execution_id or self._finished:
            return
        self._set_enabled(False)
        self._finish(ExecutorStatus.CANCELED, "Task 2 MPPI FollowPath gate disabled")

    def cleanup(self, execution_id: str) -> None:
        if execution_id == self._execution_id:
            self._set_enabled(False)
            self._finished = True

    def _finish(self, status: ExecutorStatus, message: str) -> None:
        if self._finished:
            return
        self._finished = True
        if self._complete:
            self._complete(ExecutionResult(status, message))
