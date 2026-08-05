"""Shift timing state machine."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import numpy as np

from .synchronizer import Synchronizer


class ShiftPhase(str, Enum):
    IDLE = "idle"
    CUT = "cut"                 # ignition/throttle cut
    DISENGAGE = "disengage"     # clutch out
    SELECT = "select"           # move gate / sequential
    SYNC = "sync"
    ENGAGE = "engage"           # clutch in
    COMPLETE = "complete"


@dataclass
class ShiftState:
    phase: ShiftPhase = ShiftPhase.IDLE
    current_gear: int = 0
    target_gear: int = 0
    timer: float = 0.0
    in_progress: bool = False
    ignition_cut: bool = False
    auto_clutch: float | None = None  # override engagement if not None


@dataclass
class ShiftTiming:
    cut_time: float = 0.04
    disengage_time: float = 0.06
    select_time: float = 0.05
    engage_time: float = 0.08


class ShiftController:
    def __init__(self, timing: ShiftTiming | None = None, sync_time: float = 0.10):
        self.timing = timing or ShiftTiming()
        self.sync = Synchronizer(sync_time=sync_time)
        self.state = ShiftState()

    def reset(self, gear: int = 0) -> None:
        self.sync.reset()
        self.state = ShiftState(current_gear=gear, target_gear=gear)

    def request(self, gear: int) -> None:
        if self.state.in_progress:
            return
        if gear == self.state.current_gear:
            return
        self.state.target_gear = gear
        self.state.phase = ShiftPhase.CUT
        self.state.timer = 0.0
        self.state.in_progress = True
        self.state.ignition_cut = True
        self.state.auto_clutch = 0.0

    def step(
        self,
        dt: float,
        omega_engine: float,
        omega_gearbox: float,
        driver_clutch: float,
    ) -> ShiftState:
        st = self.state
        if not st.in_progress:
            st.auto_clutch = None
            st.ignition_cut = False
            return st

        st.timer += dt
        t = self.timing

        if st.phase == ShiftPhase.CUT:
            st.ignition_cut = True
            st.auto_clutch = 0.0
            if st.timer >= t.cut_time:
                st.phase = ShiftPhase.DISENGAGE
                st.timer = 0.0

        elif st.phase == ShiftPhase.DISENGAGE:
            st.auto_clutch = 0.0
            if st.timer >= t.disengage_time:
                st.phase = ShiftPhase.SELECT
                st.timer = 0.0
                # Neutral during select
                st.current_gear = 0

        elif st.phase == ShiftPhase.SELECT:
            st.auto_clutch = 0.0
            if st.timer >= t.select_time:
                st.phase = ShiftPhase.SYNC
                st.timer = 0.0
                self.sync.begin(st.target_gear)

        elif st.phase == ShiftPhase.SYNC:
            st.auto_clutch = 0.0
            # Target gearbox input speed if locked to wheels unknown → use engine match
            self.sync.step(dt, omega_engine, omega_gearbox)
            if self.sync.state.locked or st.timer >= self.sync.sync_time * 1.5:
                st.current_gear = st.target_gear
                st.phase = ShiftPhase.ENGAGE
                st.timer = 0.0

        elif st.phase == ShiftPhase.ENGAGE:
            # Ramp clutch in
            frac = min(1.0, st.timer / max(t.engage_time, 1e-4))
            st.auto_clutch = frac
            st.ignition_cut = False
            if st.timer >= t.engage_time:
                st.phase = ShiftPhase.COMPLETE
                st.timer = 0.0

        elif st.phase == ShiftPhase.COMPLETE:
            st.in_progress = False
            st.phase = ShiftPhase.IDLE
            st.auto_clutch = None
            st.ignition_cut = False

        return st
