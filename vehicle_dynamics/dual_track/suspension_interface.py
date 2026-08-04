"""
Phase 6.2–6.5 – Suspension geometry interface.

δ_eff = δ_cmd + toe_static + toe_bump
camber_total = camber_static + camber_gain   (diagnostic)
RC_front/rear from geometric migration        (diagnostic)

No jacking forces, no tire-model changes.
"""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from ..suspension.geometry_state import VehicleGeometryState, WheelGeometryState
from ..suspension.coupling import CoupledSuspension
from ..suspension.bump_steer import BumpSteerModel
from ..suspension.bump_state import BumpSteerParams, BumpSteerState
from ..suspension.camber_gain import CamberGainModel
from ..suspension.camber_state import CamberGainParams, CamberState
from ..suspension.roll_center import RollCenterModel
from ..suspension.roll_center_state import RollCenterState


@dataclass
class SuspensionInterfaceConfig:
    enabled: bool = False
    use_geometry_solver: bool = False
    bump_steer_enabled: bool = False
    camber_gain_enabled: bool = False
    roll_center_enabled: bool = False


class SuspensionInterface:
    def __init__(
        self,
        config: SuspensionInterfaceConfig = None,
        geometry: VehicleGeometryState = None,
        coupled: CoupledSuspension = None,
        bump_params: BumpSteerParams = None,
        camber_params: CamberGainParams = None,
        roll_center_model: RollCenterModel = None,
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

        self.bump = BumpSteerModel(bump_params or BumpSteerParams.neutral())
        self.camber = CamberGainModel(camber_params or CamberGainParams.neutral())
        self.roll_center = roll_center_model or RollCenterModel()
        self._last_bump: BumpSteerState = BumpSteerState()
        self._last_camber: CamberState = CamberState()
        self._last_rc: RollCenterState = self.roll_center.last

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

    def set_wheel_travel(self, wheel_travel: np.ndarray) -> None:
        z = np.asarray(wheel_travel, dtype=float).reshape(4)
        toe_static = self._geometry.toe_array()
        camber_static = self._geometry.camber_array()

        if self.config.enabled and self.config.bump_steer_enabled:
            self._last_bump = self.bump.evaluate(z, toe_static)
        else:
            self._last_bump = BumpSteerState(
                wheel_travel=z.copy(),
                toe_bump=np.zeros(4),
                toe_static=toe_static.copy(),
                toe_total=toe_static.copy(),
            )

        if self.config.enabled and self.config.camber_gain_enabled:
            self._last_camber = self.camber.evaluate(z, camber_static)
        else:
            self._last_camber = CamberState(
                wheel_travel=z.copy(),
                camber_static=camber_static.copy(),
                camber_gain=np.zeros(4),
                camber_total=camber_static.copy(),
            )

        if self.config.enabled and self.config.roll_center_enabled:
            self._last_rc = self.roll_center.evaluate(z)
        else:
            # static RC only
            self._last_rc = self.roll_center.evaluate(np.zeros(4))
            self._last_rc.wheel_travel = z.copy()

    def effective_steer(
        self,
        delta_fl: float,
        delta_fr: float,
        delta_rl: float = 0.0,
        delta_rr: float = 0.0,
        wheel_travel: np.ndarray | None = None,
    ) -> np.ndarray:
        if not self.config.enabled:
            return np.array([delta_fl, delta_fr, delta_rl, delta_rr], dtype=float)

        if wheel_travel is not None:
            self.set_wheel_travel(wheel_travel)

        toe_static = self._geometry.toe_array()
        toe_bump = (
            self._last_bump.toe_bump
            if self.config.bump_steer_enabled
            else np.zeros(4)
        )
        toe = toe_static + toe_bump
        return np.array([
            delta_fl + toe[0],
            delta_fr + toe[1],
            delta_rl + toe[2],
            delta_rr + toe[3],
        ], dtype=float)

    def camber_total_array(self) -> np.ndarray:
        return self._last_camber.camber_total.copy()

    def diagnostics(self) -> dict:
        g = self._geometry
        d = {
            "enabled": self.config.enabled,
            "bump_steer_enabled": self.config.bump_steer_enabled,
            "camber_gain_enabled": self.config.camber_gain_enabled,
            "roll_center_enabled": self.config.roll_center_enabled,
            "camber_rad": g.camber_array().tolist(),
            "toe_static_rad": g.toe_array().tolist(),
            "Kw": g.Kw_array().tolist(),
            "Cw": g.Cw_array().tolist(),
            "kpi_rad": [g.fl.kpi_rad, g.fr.kpi_rad, g.rl.kpi_rad, g.rr.kpi_rad],
            "caster_rad": [
                g.fl.caster_rad, g.fr.caster_rad,
                g.rl.caster_rad, g.rr.caster_rad,
            ],
        }
        d.update(self._last_bump.diagnostics())
        d.update(self._last_camber.diagnostics())
        d.update(self._last_rc.diagnostics())
        return d

    def vertical_forces(
        self,
        compression: np.ndarray,
        compression_rate: np.ndarray,
    ) -> np.ndarray:
        Kw = self._geometry.Kw_array()
        Cw = self._geometry.Cw_array()
        return Kw * np.asarray(compression, dtype=float) + Cw * np.asarray(
            compression_rate, dtype=float
        )
