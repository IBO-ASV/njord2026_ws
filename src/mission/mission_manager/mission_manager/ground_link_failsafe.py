"""Ground-heartbeat loss evaluator used before starting a return-home mission."""

from __future__ import annotations

from dataclasses import dataclass
from math import log, sqrt
from typing import Optional

from .geodesy import HomeDatum, wgs84_to_enu


@dataclass(frozen=True)
class FailsafeDecision:
    trigger_return_home: bool
    status: str


class GroundLinkReturnEvaluator:
    """Arms after a heartbeat, samples from 20--30 s, then evaluates once."""

    def __init__(
        self,
        datum: HomeDatum,
        *,
        sample_start_sec: float = 20.0,
        trigger_sec: float = 30.0,
        max_position_stddev_m: float = 1.0,
        max_heading_stddev_rad: float = 0.08726646259971647,
        min_samples: int = 10,
        min_coverage_sec: float = 9.0,
        max_sample_age_sec: float = 1.5,
    ) -> None:
        self._datum = datum
        self._sample_start_sec = sample_start_sec
        self._trigger_sec = trigger_sec
        self._max_position_stddev_m = max_position_stddev_m
        self._max_heading_stddev_rad = max_heading_stddev_rad
        self._min_samples = min_samples
        self._min_coverage_sec = min_coverage_sec
        self._max_sample_age_sec = max_sample_age_sec
        self._last_heartbeat: Optional[float] = None
        self._positions: list[tuple[float, float, float]] = []
        self._headings: list[tuple[float, float]] = []
        self._evaluated = False

    def heartbeat(self, now: float) -> None:
        self._last_heartbeat = now
        self._positions.clear()
        self._headings.clear()
        self._evaluated = False

    def position(self, now: float, latitude: float, longitude: float) -> None:
        if self._sampling(now):
            east, north = wgs84_to_enu(latitude, longitude, self._datum)
            self._positions.append((now, east, north))

    def heading(self, now: float, yaw: float) -> None:
        if self._sampling(now):
            self._headings.append((now, yaw))

    def evaluate(self, now: float) -> Optional[FailsafeDecision]:
        if self._last_heartbeat is None:
            return None
        elapsed = now - self._last_heartbeat
        if elapsed < self._sample_start_sec:
            return FailsafeDecision(False, "ground heartbeat lost; waiting for 20 s sampling window")
        if elapsed < self._trigger_sec:
            return FailsafeDecision(False, "ground heartbeat lost; collecting GNSS/compass stability samples")
        if self._evaluated:
            return None
        self._evaluated = True
        if not self._complete_window(self._positions, now) or not self._complete_window(self._headings, now):
            return FailsafeDecision(False, "return-home refused: incomplete GNSS/compass 10 s sample window")
        position_stddev = self._position_stddev()
        heading_stddev = self._heading_stddev()
        if position_stddev > self._max_position_stddev_m:
            return FailsafeDecision(False, f"return-home refused: position stddev {position_stddev:.2f} m exceeds limit")
        if heading_stddev > self._max_heading_stddev_rad:
            return FailsafeDecision(False, f"return-home refused: heading stddev {heading_stddev:.3f} rad exceeds limit")
        return FailsafeDecision(True, "ground heartbeat lost for 30 s; GNSS/compass stable, requesting return-home")

    def _sampling(self, now: float) -> bool:
        return self._last_heartbeat is not None and self._sample_start_sec <= now - self._last_heartbeat <= self._trigger_sec

    def _complete_window(self, samples, now: float) -> bool:
        return (
            len(samples) >= self._min_samples
            and samples[-1][0] >= now - self._max_sample_age_sec
            and samples[-1][0] - samples[0][0] >= self._min_coverage_sec
        )

    def _position_stddev(self) -> float:
        east_mean = sum(sample[1] for sample in self._positions) / len(self._positions)
        north_mean = sum(sample[2] for sample in self._positions) / len(self._positions)
        variance = sum((sample[1] - east_mean) ** 2 + (sample[2] - north_mean) ** 2 for sample in self._positions) / len(self._positions)
        return sqrt(variance)

    def _heading_stddev(self) -> float:
        import math
        sin_mean = sum(math.sin(sample[1]) for sample in self._headings) / len(self._headings)
        cos_mean = sum(math.cos(sample[1]) for sample in self._headings) / len(self._headings)
        resultant = min(1.0, max(1e-12, sqrt(sin_mean * sin_mean + cos_mean * cos_mean)))
        return sqrt(-2.0 * log(resultant))
