"""Send route waypoints to Nav2 one goal at a time."""

from __future__ import annotations

from typing import Callable

from .base import CompletionCallback, ExecutorStatus, FeedbackCallback, TaskExecutor
from ..waypoint_config import Route


class WaypointSequenceExecutor(TaskExecutor):
    def start(
        self, execution_id: str, route: Route, feedback: FeedbackCallback, complete: CompletionCallback
    ) -> None:
        if not route.waypoints:
            complete_status = ExecutorStatus.INTERNAL_ERROR
            complete_result = "route has no waypoints"
            self._begin(execution_id, feedback, complete)
            self._finish(complete_status, complete_result)
            return
        self._begin(execution_id, feedback, complete)
        expected_id = execution_id
        waypoint_index = 0

        def send_next() -> None:
            nonlocal waypoint_index
            if expected_id != self.execution_id or self._finished:
                return
            if waypoint_index >= len(route.waypoints):
                self._report("navigate", 1.0, "waypoint route complete")
                self._finish(ExecutorStatus.SUCCEEDED, "waypoint route complete")
                return

            waypoint = route.waypoints[waypoint_index]
            ordinal = waypoint_index + 1
            self._report(
                "navigate", waypoint_index / len(route.waypoints),
                f"sending waypoint {ordinal}/{len(route.waypoints)}: {waypoint.waypoint_id}",
            )

            def accepted(ok: bool) -> None:
                if expected_id != self.execution_id or self._finished:
                    return
                if not ok:
                    self._finish(ExecutorStatus.REJECTED, "Nav2 rejected waypoint goal")
                else:
                    self._report(
                        "navigate", (waypoint_index + 0.05) / len(route.waypoints),
                        f"Nav2 accepted waypoint {ordinal}/{len(route.waypoints)}",
                    )

            def completed(status: ExecutorStatus, message: str) -> None:
                nonlocal waypoint_index
                if expected_id != self.execution_id or self._finished:
                    return
                if self._cancel_requested:
                    self._finish(ExecutorStatus.CANCELED, "waypoint goal canceled")
                elif status == ExecutorStatus.SUCCEEDED:
                    waypoint_index += 1
                    send_next()
                else:
                    self._finish(status, message or "waypoint navigation failed")

            self._navigation.send((waypoint,), accepted, completed)

        send_next()
