"""Deterministic executor used for API smoke tests without Nav2."""

from .base import CompletionCallback, ExecutorStatus, FeedbackCallback


class DummyExecutor:
    def start(self, execution_id: str, feedback: FeedbackCallback, complete: CompletionCallback) -> None:
        feedback("dummy", 0.0, "dummy executor started")
        feedback("dummy", 1.0, "dummy executor complete")
        complete(type("Result", (), {
            "status": ExecutorStatus.SUCCEEDED,
            "message": "dummy executor complete",
        })())

    def cancel(self, _execution_id: str) -> None:
        return None

    def cleanup(self, _execution_id: str) -> None:
        return None
