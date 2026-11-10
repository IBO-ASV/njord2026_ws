"""Small callback-based contract used by Mission Manager task executors."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Callable, Protocol, Sequence

from ..waypoint_config import Waypoint


class ExecutorStatus(Enum):
    SUCCEEDED = "succeeded"
    CANCELED = "canceled"
    REJECTED = "rejected"
    NAVIGATION_FAILED = "navigation_failed"
    INTERNAL_ERROR = "internal_error"


@dataclass(frozen=True)
class ExecutionResult:
    status: ExecutorStatus
    message: str


FeedbackCallback = Callable[[str, float, str], None]
CompletionCallback = Callable[[ExecutionResult], None]


class NavigationClient(Protocol):
    """Adapter implemented by the ROS NavigateThroughPoses client."""

    def send(
        self,
        poses: Sequence[Waypoint],
        accepted: Callable[[bool], None],
        completed: Callable[[ExecutorStatus, str], None],
    ) -> None:
        ...

    def cancel(self) -> None:
        ...


class TaskExecutor:
    """Base class for cancellable task behavior.

    Every callback is tagged with an execution id, so the manager can discard
    delayed Nav2 activity from a prior execution.
    """

    def __init__(self, navigation: NavigationClient) -> None:
        self._navigation = navigation
        self._execution_id = ""
        self._feedback: FeedbackCallback | None = None
        self._complete: CompletionCallback | None = None
        self._cancel_requested = False
        self._finished = False

    @property
    def execution_id(self) -> str:
        return self._execution_id

    def cancel(self, execution_id: str) -> None:
        if execution_id != self._execution_id or self._finished:
            return
        self._cancel_requested = True
        self._navigation.cancel()

    def cleanup(self, execution_id: str) -> None:
        if execution_id == self._execution_id:
            self._navigation.cancel()
            self._finished = True

    def _begin(self, execution_id: str, feedback: FeedbackCallback, complete: CompletionCallback) -> None:
        self._execution_id = execution_id
        self._feedback = feedback
        self._complete = complete
        self._cancel_requested = False
        self._finished = False

    def _report(self, stage: str, progress: float, message: str) -> None:
        if not self._finished and self._feedback:
            self._feedback(stage, progress, message)

    def _finish(self, status: ExecutorStatus, message: str) -> None:
        if self._finished:
            return
        self._finished = True
        if self._complete:
            self._complete(ExecutionResult(status, message))
