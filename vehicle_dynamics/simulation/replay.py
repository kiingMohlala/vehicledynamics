"""Replay recorded simulations."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .telemetry_recorder import TelemetryRecorder, SimSample


@dataclass
class ReplayBuffer:
    recorder: TelemetryRecorder

    def sample_at(self, time: float) -> SimSample | None:
        if not self.recorder.samples:
            return None
        times = np.array([s.time for s in self.recorder.samples])
        i = int(np.argmin(np.abs(times - time)))
        return self.recorder.samples[i]

    def matches(self, other: TelemetryRecorder, tol: float = 1e-9) -> bool:
        if len(self.recorder.samples) != len(other.samples):
            return False
        for a, b in zip(self.recorder.samples, other.samples):
            if abs(a.vx - b.vx) > tol or abs(a.x - b.x) > tol:
                return False
        return True
