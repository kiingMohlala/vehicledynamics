"""Gear synchronizer / RPM matching delay."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class SyncState:
    active: bool = False
    progress: float = 0.0      # 0..1
    target_gear: int = 0
    locked: bool = False


class Synchronizer:
    def __init__(self, sync_time: float = 0.12):
        self.sync_time = sync_time
        self.state = SyncState()

    def reset(self) -> None:
        self.state = SyncState()

    def begin(self, target_gear: int) -> None:
        self.state = SyncState(active=True, progress=0.0, target_gear=target_gear, locked=False)

    def step(self, dt: float, omega_in: float, omega_out_target: float) -> SyncState:
        if not self.state.active:
            return self.state
        # Progress based on time + RPM error reduction
        self.state.progress = min(1.0, self.state.progress + dt / max(self.sync_time, 1e-4))
        rpm_err = abs(omega_in - omega_out_target)
        if self.state.progress >= 1.0 or rpm_err < 5.0:
            self.state.locked = True
            self.state.active = False
            self.state.progress = 1.0
        return self.state
