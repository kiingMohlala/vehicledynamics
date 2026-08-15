"""
Phase 15.4 — Closed-loop ESC.

Observation → Decision → Allocation → plant.esc_brake_add

Enable flag defaults OFF so passive regression is preserved.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from vehicle_dynamics.controls.esc_observability import (
    ESCObservability,
    ReferenceYawConfig,
)
from vehicle_dynamics.controls.esc_decision import ESCDecisionLogic, ESCDecisionConfig
from vehicle_dynamics.controls.esc_command import (
    ESCCommand,
    BrakeAllocator,
    BrakeAllocatorConfig,
)


@dataclass
class ClosedLoopESCConfig:
    enabled: bool = False  # default OFF — passive plant equivalence
    K_us: float = 0.0065
    wheelbase: float = 2.70
    track_f: float = 1.65
    track_r: float = 1.62
    brake_torque_max: float = 2800.0
    # Decision
    e_enter: float = 0.12
    e_exit: float = 0.06
    K_Mz: float = 4000.0
    max_delta_Mz: float = 6000.0
    min_vx: float = 8.0
    min_delta: float = 0.015
    max_util: float = 0.98
    max_beta: float = 0.45


class ClosedLoopESC:
    """
    Thin orchestration. Does not alter plant physics — only sets esc_brake_add.
    """

    def __init__(self, cfg: ClosedLoopESCConfig | None = None):
        self.cfg = cfg or ClosedLoopESCConfig()
        self.observer = ESCObservability(ReferenceYawConfig(
            wheelbase=self.cfg.wheelbase,
            K_us=self.cfg.K_us,
        ))
        self.decision = ESCDecisionLogic(ESCDecisionConfig(
            e_enter=self.cfg.e_enter,
            e_exit=self.cfg.e_exit,
            K_Mz=self.cfg.K_Mz,
            max_delta_Mz=self.cfg.max_delta_Mz,
            min_vx=self.cfg.min_vx,
            min_delta=self.cfg.min_delta,
            max_util=self.cfg.max_util,
            max_beta=self.cfg.max_beta,
        ))
        self.allocator = BrakeAllocator(BrakeAllocatorConfig(
            track_f=self.cfg.track_f,
            track_r=self.cfg.track_r,
            brake_torque_max=self.cfg.brake_torque_max,
            max_delta_Mz=self.cfg.max_delta_Mz,
        ))
        self.last_Mz = 0.0
        self.last_active = False

    def reset(self) -> None:
        self.observer.reset()
        self.decision.reset()
        self.last_Mz = 0.0
        self.last_active = False

    def step(self, sim) -> np.ndarray:
        """
        Observe sim, decide, allocate. Returns brake_add[4].
        If disabled → zeros (and clears sim.esc_brake_add).
        """
        if not self.cfg.enabled:
            add = np.zeros(4)
            sim.esc_brake_add = add
            self.last_Mz = 0.0
            self.last_active = False
            return add

        obs = self.observer.observe_from_simulation(sim)
        dec = self.decision.step(obs)
        if not dec.active or abs(dec.delta_Mz_request) < 1e-6:
            add = np.zeros(4)
            self.last_Mz = 0.0
            self.last_active = False
        else:
            alloc = self.allocator.allocate(ESCCommand(dec.delta_Mz_request))
            add = np.asarray(alloc.brake_cmd, dtype=float)
            self.last_Mz = float(dec.delta_Mz_request)
            self.last_active = True
        sim.esc_brake_add = add
        return add
