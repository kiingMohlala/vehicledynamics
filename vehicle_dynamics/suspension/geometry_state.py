"""
Per-wheel geometry state exposed to the dual-track vehicle.

Camber / KPI / caster / roll-center are diagnostic in Phase 6.2.
Toe is the only geometric quantity that affects wheel heading.
Kw / Cw replace fixed spring/damper rates when the interface is enabled.
"""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class WheelGeometryState:
    """State for one wheel (FL, FR, RL, or RR)."""
    camber_rad: float = 0.0       # diagnostic only (Phase 6.2)
    toe_rad: float = 0.0          # added to steer command
    kpi_rad: float = 0.0          # diagnostic
    caster_rad: float = 0.0       # diagnostic
    scrub_radius: float = 0.0     # diagnostic
    trail: float = 0.0            # diagnostic
    roll_center_z: float = 0.0    # diagnostic
    installation_ratio: float = 1.0
    motion_ratio: float = 1.0
    Kw: float = 30000.0           # effective wheel rate [N/m]
    Cw: float = 2000.0            # effective wheel damping [N·s/m]

    @classmethod
    def neutral(cls, Kw: float = 30000.0, Cw: float = 2000.0) -> "WheelGeometryState":
        """Zero offsets, IR=1 — must match Phase 5 baseline behaviour."""
        return cls(
            camber_rad=0.0,
            toe_rad=0.0,
            kpi_rad=0.0,
            caster_rad=0.0,
            scrub_radius=0.0,
            trail=0.0,
            roll_center_z=0.0,
            installation_ratio=1.0,
            motion_ratio=1.0,
            Kw=Kw,
            Cw=Cw,
        )


@dataclass
class VehicleGeometryState:
    """Four-wheel geometry snapshot."""
    fl: WheelGeometryState
    fr: WheelGeometryState
    rl: WheelGeometryState
    rr: WheelGeometryState

    def as_list(self) -> list[WheelGeometryState]:
        return [self.fl, self.fr, self.rl, self.rr]

    def toe_array(self) -> np.ndarray:
        return np.array([w.toe_rad for w in self.as_list()])

    def camber_array(self) -> np.ndarray:
        return np.array([w.camber_rad for w in self.as_list()])

    def Kw_array(self) -> np.ndarray:
        return np.array([w.Kw for w in self.as_list()])

    def Cw_array(self) -> np.ndarray:
        return np.array([w.Cw for w in self.as_list()])

    @classmethod
    def neutral(cls, Kw: float = 30000.0, Cw: float = 2000.0) -> "VehicleGeometryState":
        n = WheelGeometryState.neutral(Kw, Cw)
        return cls(fl=n, fr=n, rl=n, rr=n)
