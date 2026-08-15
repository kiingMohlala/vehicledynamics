"""
Phase 15.2 — ESC differential-brake command authority.

COMMAND PATH ONLY — no feedback on e_r, no decision logic.

ESCCommand(requested ΔMz)
        ↓
BrakeAllocator  (geometry + limits, independent of e_r)
        ↓
wheel brake torque requests [FL, FR, RL, RR]
        ↓
frozen plant
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class ESCCommand:
    """
    Requested corrective yaw moment [N·m].

    Sign convention (vehicle frame):
      +Mz = yaw left (CCW)  → brake left-side wheels (plant geometry)
      −Mz = yaw right (CW)  → brake right-side wheels
    """
    delta_Mz: float = 0.0


@dataclass
class BrakeAllocatorConfig:
    track_f: float = 1.65
    track_r: float = 1.62
    brake_torque_max: float = 2800.0
    # Fraction of |ΔMz| allocated to front vs rear axle (passive bias)
    front_share: float = 0.55
    # Hard limits
    max_wheel_brake_cmd: float = 1.0   # fraction of brake_torque_max
    max_delta_Mz: float = 8000.0       # N·m request clamp


@dataclass
class BrakeAllocation:
    """Per-wheel brake command as fraction of brake_torque_max [0, 1]."""
    brake_cmd: np.ndarray = field(default_factory=lambda: np.zeros(4))
    achieved_Mz: float = 0.0
    requested_Mz: float = 0.0

    @property
    def fl(self) -> float:
        return float(self.brake_cmd[0])

    @property
    def fr(self) -> float:
        return float(self.brake_cmd[1])

    @property
    def rl(self) -> float:
        return float(self.brake_cmd[2])

    @property
    def rr(self) -> float:
        return float(self.brake_cmd[3])


class BrakeAllocator:
    """
    Map requested ΔMz → individual brake fractions.

    Purely geometric / limit-based. Does NOT read e_r, β, or ay.
    Does NOT generate drive torque.
    """

    def __init__(self, cfg: BrakeAllocatorConfig | None = None):
        self.cfg = cfg or BrakeAllocatorConfig()

    def allocate(self, cmd: ESCCommand) -> BrakeAllocation:
        cfg = self.cfg
        M_req = float(np.clip(cmd.delta_Mz, -cfg.max_delta_Mz, cfg.max_delta_Mz))
        out = np.zeros(4, dtype=float)

        if abs(M_req) < 1e-6:
            return BrakeAllocation(brake_cmd=out, achieved_Mz=0.0, requested_Mz=0.0)

        # Plant geometry: FL at +y, FR at −y.
        # Braking left (FL/RL) → +Mz (CCW); braking right → −Mz.
        # +Mz request → left side; −Mz request → right side.
        side = -1 if M_req > 0 else 1  # -1 → left side (indices 0,2)
        M_abs = abs(M_req)
        Mf = cfg.front_share * M_abs
        Mr = (1.0 - cfg.front_share) * M_abs

        # Mz ≈ F_brake * (track/2); F_brake = T_brake / r_wheel ≈ cmd * T_max / r
        # We work in torque domain: desired brake torque on axle side
        # M ≈ (T_brake / r) * (track/2) → T_brake = M * 2 * r / track
        # Without explicit r here, use torque-proxy: scale by track geometry
        # T_side = 2 * M / track  (effective lever arm track/2 → factor 2/track)
        # Then normalize by brake_torque_max to get cmd fraction.
        Tf = (2.0 * Mf) / max(cfg.track_f, 0.1)
        Tr = (2.0 * Mr) / max(cfg.track_r, 0.1)

        cmd_f = float(np.clip(Tf / max(cfg.brake_torque_max, 1.0), 0.0, cfg.max_wheel_brake_cmd))
        cmd_r = float(np.clip(Tr / max(cfg.brake_torque_max, 1.0), 0.0, cfg.max_wheel_brake_cmd))

        if side > 0:
            out[1] = cmd_f  # FR
            out[3] = cmd_r  # RR
        else:
            out[0] = cmd_f  # FL
            out[2] = cmd_r  # RL

        # Achieved moment estimate (same model inverted)
        # +Mz from right brakes: +(T_fr * track_f/2 + T_rr * track_r/2) / r_proxy
        # Using T = cmd * T_max and lever track/2 → M ≈ cmd * T_max * track/2
        # (consistent with T = 2M/track)
        # Plant: FL(+y)/RL brake → +Mz; FR/RR → −Mz
        M_ach = 0.0
        M_ach += out[0] * cfg.brake_torque_max * (cfg.track_f / 2.0)
        M_ach += out[2] * cfg.brake_torque_max * (cfg.track_r / 2.0)
        M_ach -= out[1] * cfg.brake_torque_max * (cfg.track_f / 2.0)
        M_ach -= out[3] * cfg.brake_torque_max * (cfg.track_r / 2.0)

        return BrakeAllocation(
            brake_cmd=out,
            achieved_Mz=float(M_ach),
            requested_Mz=M_req,
        )
