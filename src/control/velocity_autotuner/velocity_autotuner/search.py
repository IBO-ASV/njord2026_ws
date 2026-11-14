"""Bounded one-parameter search used by the ROS velocity tuner."""
from __future__ import annotations
import math

class BoundedSearch:
    """Bisection for signed steady error; interval refinement for transient loss."""
    def __init__(self, lower: float, upper: float, rounds: int, mode: str) -> None:
        if not (math.isfinite(lower) and math.isfinite(upper) and 0.0 <= lower < upper):
            raise ValueError("bounds must be finite, nonnegative, and increasing")
        if rounds < 1 or mode not in {"steady", "loss"}:
            raise ValueError("invalid search rounds or mode")
        self.lower, self.upper = lower, upper
        self.initial_low, self.initial_high = lower, upper
        self.rounds, self.mode = rounds, mode
        self.results: dict[float, tuple[float, float]] = {}
        self.pending = [lower, upper]
        self.midpoints = 0
        self.done = False

    def next_candidate(self) -> float | None:
        return self.pending[0] if self.pending else None

    def record(self, value: float, score: float, signed_error: float = 0.0) -> None:
        if not self.pending or not math.isclose(self.pending[0], value, rel_tol=0.0, abs_tol=1e-12):
            raise ValueError("unexpected candidate result")
        self.pending.pop(0)
        if not math.isfinite(score) or not math.isfinite(signed_error):
            score = math.inf
        self.results[value] = (score, signed_error)
        if self.pending:
            return
        if self.mode == "steady":
            self._advance_steady()
        else:
            self._advance_loss()

    def _schedule_midpoint(self) -> None:
        candidate = (self.lower + self.upper) / 2.0
        if any(math.isclose(candidate, tested, rel_tol=0.0, abs_tol=1e-12) for tested in self.results):
            self.done = True
            return
        self.pending = [candidate]
        self.midpoints += 1

    def _advance_steady(self) -> None:
        if len(self.results) == 2:
            lo, hi = self.initial_low, self.initial_high
            err_lo, err_hi = self.results[lo][1], self.results[hi][1]
            if err_lo == 0.0 or err_hi == 0.0 or err_lo * err_hi > 0.0:
                self.done = True
                return
        else:
            midpoint = next(v for v in self.results if self.lower < v < self.upper)
            err_mid = self.results[midpoint][1]
            err_low = self.results[self.lower][1]
            if err_mid == 0.0:
                self.done = True
                return
            if err_low * err_mid <= 0.0:
                self.upper = midpoint
            else:
                self.lower = midpoint
        if self.midpoints >= self.rounds:
            self.done = True
        else:
            self._schedule_midpoint()

    def _advance_loss(self) -> None:
        if len(self.results) == 2 and self.midpoints == 0:
            self._schedule_midpoint()
            return
        if self.midpoints >= self.rounds:
            self.done = True
            return
        middle_value = next(v for v in self.results if self.lower < v < self.upper)
        left_score = self.results[self.lower][0]
        mid_score = self.results[middle_value][0]
        right_score = self.results[self.upper][0]
        if mid_score <= left_score and mid_score <= right_score:
            if left_score <= right_score:
                self.upper = middle_value
            else:
                self.lower = middle_value
        elif left_score <= mid_score:
            self.upper = middle_value
        else:
            self.lower = middle_value
        self._schedule_midpoint()

    def best_candidate(self) -> float:
        return self.best()[0] if self.best() else self.initial_low

    def best(self) -> tuple[float, float, float] | None:
        if not self.results:
            return None
        value, (score, error) = min(self.results.items(), key=lambda item: item[1][0])
        return value, score, error
