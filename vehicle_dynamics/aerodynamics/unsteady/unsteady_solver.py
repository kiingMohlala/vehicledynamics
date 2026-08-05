"""
Unsteady aero solver: gust + crosswind + wake + transient lag
on top of quasi-steady ClosedLoop / analytical aero.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np

from vehicle_dynamics.aerodynamics.coefficients import AeroConfig
from vehicle_dynamics.aerodynamics.ride_height import RideHeightState
from vehicle_dynamics.aerodynamics.aero_model import AeroState, compute_aero_loads
from vehicle_dynamics.aerodynamics.aero_solver import AeroResult, solve_aero

from .dynamic_pressure import relative_velocity, dynamic_pressure_rel, air_speed_and_sideslip
from .gust_model import GustModel, StepGust
from .crosswind import compute_crosswind_loads
from .wake_model import WakeField, evaluate_wake
from .aero_transients import AeroTransientFilter
from .wake_database import WakeDatabase


@dataclass
class UnsteadyAeroConfig:
    enabled: bool = True
    rho: float = 1.225
    # When False → pure quasi-steady Phase 9.3 path
    use_gust: bool = True
    use_crosswind: bool = True
    use_wake: bool = True
    use_transients: bool = True


@dataclass
class UnsteadyAeroState:
    aero: AeroResult
    wind: np.ndarray
    v_rel: np.ndarray
    airspeed: float
    beta_aero: float
    wake_strength: float
    Cd_factor: float
    Cl_factor: float
    source: str = "unsteady"


class UnsteadyAeroSolver:
    def __init__(
        self,
        aero_cfg: AeroConfig | None = None,
        unsteady_cfg: UnsteadyAeroConfig | None = None,
        gust: GustModel | None = None,
        wake_db: WakeDatabase | None = None,
    ):
        self.aero_cfg = aero_cfg or AeroConfig()
        self.ucfg = unsteady_cfg or UnsteadyAeroConfig()
        self.gust = gust
        self.wake_db = wake_db or WakeDatabase()
        self.filter = AeroTransientFilter()
        self._t = 0.0

    def reset(self) -> None:
        self._t = 0.0
        self.filter.reset()
        if hasattr(self.gust, "reset"):
            self.gust.reset()  # type: ignore

    def step(
        self,
        speed: float,
        ride: RideHeightState | None = None,
        *,
        dt: float = 0.01,
        yaw_rate: float = 0.0,
        ego_x: float = 0.0,
        ego_y: float = 0.0,
        v_vehicle: np.ndarray | None = None,
    ) -> UnsteadyAeroState:
        """
        Advance unsteady aero by dt.

        speed: nominal longitudinal speed (used if v_vehicle not given).
        """
        cfg = self.aero_cfg
        ride = ride or RideHeightState(h_front=cfg.h_front_ref, h_rear=cfg.h_rear_ref)
        self._t += dt

        # --- Regression path ---
        if not self.ucfg.enabled:
            aero = solve_aero(speed, cfg=cfg, ride=ride)
            z = np.zeros(3)
            return UnsteadyAeroState(
                aero=aero, wind=z, v_rel=np.array([speed, 0.0, 0.0]),
                airspeed=speed, beta_aero=0.0, wake_strength=0.0,
                Cd_factor=1.0, Cl_factor=1.0, source="steady",
            )

        # Wind
        wind = np.zeros(3)
        if self.ucfg.use_gust and self.gust is not None:
            wind = np.asarray(self.gust.wind(self._t), dtype=float)

        if v_vehicle is None:
            v_vehicle = np.array([speed, 0.0, 0.0])
        v_rel = relative_velocity(v_vehicle, wind)
        airspeed, beta = air_speed_and_sideslip(v_rel)
        q = dynamic_pressure_rel(self.ucfg.rho, v_rel)

        # Quasi-steady baseline at air-relative speed
        base = compute_aero_loads(airspeed, cfg, ride=ride)

        # Wake / drafting
        Cd_f = Cl_f = 1.0
        wake_strength = 0.0
        if self.ucfg.use_wake and self.wake_db.field.sources:
            wf = evaluate_wake(self.wake_db.field, ego_x=ego_x, ego_y=ego_y)
            Cd_f = wf.get("Cd_factor", 1.0)
            Cl_f = wf.get("Cl_factor", 1.0)
            wake_strength = wf.get("wake_strength", 0.0)

        Fx = base.Fx * Cd_f
        Fz_f = base.Fz_front * Cl_f
        Fz_r = base.Fz_rear * Cl_f
        Fy = base.Fy
        Mx = 0.0
        Mz = base.Mz

        # Crosswind from aerodynamic sideslip
        if self.ucfg.use_crosswind and abs(beta) > 1e-6:
            cw = compute_crosswind_loads(
                q, beta,
                S=cfg.frontal_area,
                L=cfg.wheelbase,
                track=cfg.track,
                Cy_beta=cfg.coeffs.Cy_beta,
                Cn_beta=cfg.coeffs.Cn_yaw,
            )
            Fy = cw.Fy
            Mx = cw.Mx
            Mz = cw.Mz

        # Transient lag
        if self.ucfg.use_transients:
            Fx, Fy, Fz_f, Fz_r, Mx, Mz = self.filter.step(
                Fx, Fy, Fz_f, Fz_r, Mx, Mz, dt
            )

        # Pack AeroState / AeroResult
        S = cfg.frontal_area
        Cd_eff = -Fx / (q * S) if q * S > 1e-9 else 0.0
        Cl_fe = Fz_f / (q * S) if q * S > 1e-9 else 0.0
        Cl_re = Fz_r / (q * S) if q * S > 1e-9 else 0.0
        L = cfg.wheelbase
        a = b = 0.5 * L
        My = -Fz_f * a + Fz_r * b
        Fz_tot = Fz_f + Fz_r
        x_cp = (Fz_r * b - Fz_f * a) / Fz_tot if abs(Fz_tot) > 1e-9 else 0.0

        st = AeroState(
            q=q, Fx=Fx, Fy=Fy, Fz_front=Fz_f, Fz_rear=Fz_r,
            Mx=Mx, My=My, Mz=Mz,
            Cd_eff=Cd_eff, Cl_front_eff=Cl_fe, Cl_rear_eff=Cl_re,
            center_of_pressure_x=x_cp, cooling_drag=base.cooling_drag * Cd_f,
        )
        aero = AeroResult(
            state=st, speed=airspeed, ride=ride, config=cfg,
            dFz_front=-Fz_f, dFz_rear=-Fz_r,
            drag_force=-Fx if Fx < 0 else Fx,
            side_force=Fy,
            drag_power=max(-Fx, 0.0) * airspeed,
        )
        # drag_force positive magnitude
        aero.drag_force = st.drag

        return UnsteadyAeroState(
            aero=aero,
            wind=wind,
            v_rel=v_rel,
            airspeed=airspeed,
            beta_aero=beta,
            wake_strength=wake_strength,
            Cd_factor=Cd_f,
            Cl_factor=Cl_f,
            source="unsteady",
        )
