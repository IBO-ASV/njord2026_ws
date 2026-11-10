"""Cancelable state machine for the existing Task 3 docking stages."""

from __future__ import annotations

from typing import Callable, Optional

from .base import CompletionCallback, ExecutorStatus, FeedbackCallback, TaskExecutor
from ..waypoint_config import Route


class StagedDockingExecutor(TaskExecutor):
    """Runs gate, dock, berth wait, and exit stages without process restarts."""

    def __init__(self, navigation, create_wait_timer: Callable[[float, Callable[[], None]], object],
                 cancel_wait_timer: Callable[[object], None], dock_target_correction=None,
                 stage_succeeded=None) -> None:
        super().__init__(navigation)
        self._create_wait_timer = create_wait_timer
        self._cancel_wait_timer = cancel_wait_timer
        self._dock_target_correction = dock_target_correction
        self._stage_succeeded = stage_succeeded
        self._route: Optional[Route] = None
        self._full_sequence = False
        self._stage_names: list[str] = []
        self._stage_index = 0
        self._retry_count = 0
        self._max_retries = 0
        self._wait_timer: object | None = None
        self._berth_wait_count = 0

    def start(self, execution_id: str, route: Route, feedback: FeedbackCallback,
              complete: CompletionCallback, *, full_sequence: bool = False) -> None:
        self._begin(execution_id, feedback, complete)
        self._route = route
        self._full_sequence = full_sequence
        stage_set = route.full_sequence_stages if full_sequence else route.stages
        configured_order = route.constraints.get("stage_order")
        self._stage_names = [name for name in configured_order if stage_set.get(name)] if isinstance(configured_order, list) else [
            name for name in ("stage_1_gate", "stage_1", "stage_2", "stage_3", "stage_4", "stage_5") if stage_set.get(name)
        ]
        self._stage_index = 0
        self._retry_count = 0
        self._berth_wait_count = 0
        attempts = route.constraints.get("attempts", 1)
        self._max_retries = max(0, int(attempts) - 1) if isinstance(attempts, (int, float)) else 0
        if not self._stage_names:
            self._finish(ExecutorStatus.INTERNAL_ERROR, "Task 3 route defines no non-empty stages")
            return
        self._send_current_stage()

    def cancel(self, execution_id: str) -> None:
        if execution_id != self.execution_id:
            return
        waiting = self._wait_timer is not None
        self._cancel_wait()
        super().cancel(execution_id)
        if waiting:
            # No Nav2 result remains to drive completion once a berth timer is
            # canceled, so finish this path synchronously.
            self._finish(ExecutorStatus.CANCELED, "docking task canceled during berth wait")

    def cleanup(self, execution_id: str) -> None:
        self._cancel_wait()
        super().cleanup(execution_id)

    def _send_current_stage(self) -> None:
        if self._finished or self._route is None:
            return
        stage_name = self._stage_names[self._stage_index]
        poses = self._route.stage(stage_name, full_sequence=self._full_sequence)
        if self._dock_target_correction is not None and poses and poses[-1].waypoint_type == "dock":
            poses = (*poses[:-1], self._dock_target_correction(poses[-1]))
        progress = self._stage_index / len(self._stage_names)
        self._report(stage_name, progress, f"sending docking {stage_name}")
        expected_id = self.execution_id

        def accepted(ok: bool) -> None:
            if expected_id != self.execution_id or self._finished:
                return
            if not ok:
                self._retry_or_finish("Nav2 rejected docking goal")

        def completed(status: ExecutorStatus, message: str) -> None:
            if expected_id != self.execution_id or self._finished:
                return
            if self._cancel_requested:
                self._finish(ExecutorStatus.CANCELED, "docking task canceled")
            elif status != ExecutorStatus.SUCCEEDED:
                self._retry_or_finish(message or "docking navigation failed")
            else:
                if self._stage_succeeded is not None:
                    self._stage_succeeded(stage_name, poses)
                if self._needs_berth_wait():
                    self._start_wait()
                else:
                    self._advance_stage()

        self._navigation.send(poses, accepted, completed)

    def _needs_berth_wait(self) -> bool:
        """Wait only after a stage which actually ends at a docking berth.

        ``stage_2`` is the second berth only in the optional continuous Task
        3.1 sequence; in the normal Task 3.1 route it is the undocking/exit
        stage and must not incur another berth wait.
        """
        if self._route is None:
            return False
        stage_name = self._stage_names[self._stage_index]
        poses = self._route.stage(stage_name, full_sequence=self._full_sequence)
        return bool(poses and poses[-1].waypoint_type == "dock")

    def _start_wait(self) -> None:
        if self._route is None:
            self._finish(ExecutorStatus.INTERNAL_ERROR, "missing route during berth wait")
            return
        stage = self._stage_names[self._stage_index]
        # A continuous run visits two docks.  Use the route's optional second
        # berth dwell time for the latter without coupling this behavior to a
        # particular waypoint ID.
        seconds_key = "second_wait_time_s" if self._berth_wait_count else "wait_time_s"
        seconds = self._route.constraints.get(seconds_key, self._route.constraints.get("wait_time_s", 0.0))
        seconds = float(seconds) if isinstance(seconds, (int, float)) else 0.0
        self._berth_wait_count += 1
        self._report(f"{stage}_wait", (self._stage_index + 0.5) / len(self._stage_names),
                     f"waiting at berth for {seconds:g} seconds")
        expected_id = self.execution_id

        def elapsed() -> None:
            self._wait_timer = None
            if expected_id != self.execution_id or self._finished or self._cancel_requested:
                return
            self._advance_stage()

        self._wait_timer = self._create_wait_timer(max(0.0, seconds), elapsed)

    def _advance_stage(self) -> None:
        self._cancel_wait()
        self._stage_index += 1
        self._retry_count = 0
        if self._stage_index >= len(self._stage_names):
            self._report("complete", 1.0, "docking route complete")
            self._finish(ExecutorStatus.SUCCEEDED, "docking route complete")
        else:
            self._send_current_stage()

    def _retry_or_finish(self, message: str) -> None:
        if self._cancel_requested:
            self._finish(ExecutorStatus.CANCELED, "docking task canceled")
        elif self._retry_count >= self._max_retries:
            self._finish(ExecutorStatus.NAVIGATION_FAILED, message)
        else:
            self._retry_count += 1
            self._report(self._stage_names[self._stage_index], 0.0,
                         f"retry {self._retry_count}/{self._max_retries}: {message}")
            self._send_current_stage()

    def _cancel_wait(self) -> None:
        if self._wait_timer is not None:
            self._cancel_wait_timer(self._wait_timer)
            self._wait_timer = None
