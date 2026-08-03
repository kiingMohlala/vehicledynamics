"""
Phase 6.2 – Suspension geometry interface for the dual-track plant.

Opt-in: when disabled, behaviour matches the frozen Phase 5 baseline.
When enabled:
  · δ_eff = δ_command + toe   (orientation only)
  · Kw, Cw available for vertical force path (if used)
  · camber / KPI / caster / RC logged as diagnostics only

No camber thrust, no jacking forces, no tire-model changes.
"""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from ..suspension.geometry_state import VehicleGeometryState, WheelGeometryState
from ..suspension.coupling import CoupledSuspension, VehicleSuspensionConfig


@dataclass
class SuspensionInterfaceConfig:
    enabled: bool = False
    """If False → neutral geometry (Phase 5 equivalent)."""
    use_geometry_solver: bool = False
    """If True, build state from CoupledSuspension hardpoints."""


class SuspensionInterface:
    def __init__(
        self,
        config: SuspensionInterfaceConfig = None,
        geometry: VehicleGeometryState = None,
        coupled: CoupledSuspension = None,
    ):
        self.config = config or SuspensionInterfaceConfig()
        self._coupled = coupled
        if geometry is not None:
            self._geometry = geometry
        elif self.config.enabled and self.config.use_geometry_solver:
            self._coupled = coupled or CoupledSuspension()
            self._geometry = self._from_coupled(self._coupled)
        else:
            self._geometry = VehicleGeometryState.neutral()

    @staticmethod
    def _from_coupled(coupled: CoupledSuspension) -> VehicleGeometryState:
        states = coupled.evaluate_all()

        def to_wgs(s) -> WheelGeometryState:
            return WheelGeometryState(
                camber_rad=s.camber_rad,
                toe_rad=s.toe_rad,
                kpi_rad=s.kpi_rad,
                caster_rad=s.caster_rad,
                scrub_radius=s.scrub_radius,
                trail=s.trail,
                roll_center_z=s.roll_center_z,
                installation_ratio=s.installation_ratio,
                motion_ratio=s.motion_ratio,
                Kw=s.Kw,
                Cw=s.Cw,
            )

        return VehicleGeometryState(
            fl=to_wgs(states["FL"]),
            fr=to_wgs(states["FR"]),
            rl=to_wgs(states["RL"]),
            rr=to_wgs(states["RR"]),
        )

    @property
    def geometry(self) -> VehicleGeometryState:
        return self._geometry

    def effective_steer(
        self,
        delta_fl: float,
        delta_fr: float,
        delta_rl: float = 0.0,
        delta_rr: float = 0.0,
    ) -> np.ndarray:
        """
        Per-wheel heading = command + toe.
        Rear steer defaults to 0 + rear toe.
        """
        if not self.config.enabled:
            return np.array([delta_fl, delta_fr, delta_rl, delta_rr], dtype=float)
        toe = self._geometry.toe_array()
        return np.array([
            delta_fl + toe[0],
            delta_fr + toe[1],
            delta_rl + toe[2],
            delta_rr + toe[3],
        ], dtype=float)

    def diagnostics(self) -> dict:
        g = self._geometry
        return {
            "enabled": self.config.enabled,
            "camber_rad": g.camber_array().tolist(),
            "toe_rad": g.toe_array().tolist(),
            "Kw": g.Kw_array().tolist(),
            "Cw": g.Cw_array().tolist(),
            "roll_center_z": [g.fl.roll_center_z, g.fr.roll_center_z,
                              g.rl.roll_center_z, g.rr.roll_center_z],
            "kpi_rad": [g.fl.kpi_rad, g.fr.kpi_rad, g.rl.kpi_rad, g.rr.kpi_rad],
            "caster_rad": [g.fl.caster_rad, g.fr.caster_rad,
                           g.rl.caster_rad, g.rr.caster_rad],
        }

    def vertical_forces(
        self,
        compression: np.ndarray,
        compression_rate: np.ndarray,
    ) -> np.ndarray:
        """F = Kw*z + Cw*zdot (positive compression → positive force)."""
        Kw = self._geometry.Kw_array()
        Cw = self._geometry.Cw_array()
        return Kw * np.asarray(compression, dtype=float) + Cw * np.asarray(
            compression_rate, dtype=float
        )
