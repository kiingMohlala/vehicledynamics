"""High-level aerodynamic database with lookup modes."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
import numpy as np

from vehicle_dynamics.aerodynamics.coefficients import AeroConfig, AeroCoefficients
from vehicle_dynamics.aerodynamics.ride_height import RideHeightState
from vehicle_dynamics.aerodynamics.aero_model import AeroState, compute_aero_loads, dynamic_pressure

from .cfd_map import AeroSample, AeroMapND
from .interpolator import interpolate_sample
from .uncertainty import estimate_uncertainty, UncertaintyEstimate


class AeroSolverMode(str, Enum):
    ANALYTICAL = "analytical"
    LOOKUP = "lookup"
    HYBRID = "hybrid"


@dataclass
class AeroDatabase:
    amap: AeroMapND = field(default_factory=AeroMapND)
    mode: AeroSolverMode = AeroSolverMode.ANALYTICAL
    config: AeroConfig = field(default_factory=AeroConfig)
    max_interp_distance: float = 2.5

    def add_samples(self, samples: list[AeroSample]) -> None:
        self.amap.samples.extend(samples)

    def clear(self) -> None:
        self.amap.samples.clear()

    def lookup(
        self,
        speed: float,
        ride: RideHeightState,
        *,
        wing_angle: float = 0.12,
        drs: float = 0.0,
    ) -> tuple[AeroState, UncertaintyEstimate, str]:
        """
        Returns (AeroState, uncertainty, source) where source is
        analytical | lookup | hybrid_fallback.
        """
        cfg = self.config
        if not cfg.enabled or speed <= 0:
            return AeroState(), UncertaintyEstimate(), "analytical"

        if self.mode == AeroSolverMode.ANALYTICAL or len(self.amap) == 0:
            st = compute_aero_loads(speed, cfg, ride=ride)
            return st, UncertaintyEstimate(confidence=1.0 if self.mode == AeroSolverMode.ANALYTICAL else 0.5), "analytical"

        query = AeroSample(
            speed=speed,
            h_front=ride.h_front,
            h_rear=ride.h_rear,
            pitch=ride.pitch_rad,
            yaw=ride.yaw_rad,
            wing_angle=wing_angle,
            drs=drs,
        )
        interp, d_min, in_bounds = interpolate_sample(
            query, self.amap, max_distance=self.max_interp_distance
        )
        unc = estimate_uncertainty(d_min, len(self.amap), in_bounds)

        use_lookup = in_bounds and d_min <= self.max_interp_distance
        if self.mode == AeroSolverMode.LOOKUP and not use_lookup:
            # Out of bounds: return zeros with low confidence (protection)
            return AeroState(), unc, "lookup_oob"

        if self.mode == AeroSolverMode.HYBRID and not use_lookup:
            st = compute_aero_loads(speed, cfg, ride=ride)
            return st, unc, "hybrid_fallback"

        # Build AeroState from interpolated coeffs
        q = dynamic_pressure(cfg.rho, speed)
        S = cfg.frontal_area
        L = cfg.wheelbase
        Fx = -interp.Cd * q * S
        Fz_f = interp.Cl_front * q * S
        Fz_r = interp.Cl_rear * q * S
        Fy = interp.Cy * q * S
        a = b = 0.5 * L
        My = -Fz_f * a + Fz_r * b + interp.Cm_pitch * q * S * cfg.ref_chord
        Mz = interp.Cn_yaw * q * S * L
        st = AeroState(
            q=q,
            Fx=Fx,
            Fy=Fy,
            Fz_front=Fz_f,
            Fz_rear=Fz_r,
            Mx=0.0,
            My=My,
            Mz=Mz,
            Cd_eff=interp.Cd,
            Cl_front_eff=interp.Cl_front,
            Cl_rear_eff=interp.Cl_rear,
            center_of_pressure_x=interp.x_cop,
            cooling_drag=0.0,
        )
        return st, unc, "lookup"
