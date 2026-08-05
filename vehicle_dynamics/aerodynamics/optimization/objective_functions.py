"""Aero performance objectives from design vector (analytical plant)."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from vehicle_dynamics.aerodynamics.coefficients import AeroConfig, AeroCoefficients
from vehicle_dynamics.aerodynamics.ride_height import RideHeightState
from vehicle_dynamics.aerodynamics.aero_model import compute_aero_loads
from vehicle_dynamics.aerodynamics.aero_devices import AeroDeviceConfig
from vehicle_dynamics.aerodynamics.aero_device_solver import AeroDeviceSolver
from vehicle_dynamics.aerodynamics.wing_model import WingParams

from .design_variables import DesignVector


@dataclass
class Objectives:
    downforce: float = 0.0      # N (maximize)
    drag: float = 0.0           # N (minimize)
    L_over_D: float = 0.0       # maximize
    front_balance: float = 0.5  # target ~0.4
    stability: float = 0.0      # higher = better (rear bias soft score)
    lap_time: float = 0.0       # s (minimize)
    fuel_proxy: float = 0.0     # drag * speed proxy (minimize)


def evaluate_objectives(
    design: DesignVector,
    *,
    speed: float = 50.0,
    cfg: AeroConfig | None = None,
    use_devices: bool = True,
) -> Objectives:
    cfg = cfg or AeroConfig()
    ride = RideHeightState(
        h_front=design.h_front,
        h_rear=design.h_rear,
        pitch_rad=0.0,
    )

    if use_devices:
        dcfg = AeroDeviceConfig(
            devices_enabled=True,
            use_active_aero=False,
            use_drs=True,
            front_wing_alpha=design.front_wing_angle,
            rear_wing_alpha=design.rear_wing_angle,
            front_wing=WingParams(
                area=0.30 * design.wing_span_scale * design.chord_scale,
                Cl0=0.85 + 0.02 * design.gurney_mm,
            ),
            rear_wing=WingParams(
                area=0.40 * design.wing_span_scale * design.chord_scale,
                Cl0=1.1 + 0.015 * design.gurney_mm,
            ),
        )
        # Diffuser height via ride already; angle scales Cl mildly in solver via rake
        sol = AeroDeviceSolver(cfg, dcfg)
        # DRS schedule as partial open
        sol.drs.position = float(np.clip(design.drs_schedule, 0.0, 1.0))
        res = sol.solve(speed, ride)
        st = res.state
    else:
        st = compute_aero_loads(speed, cfg, ride=ride)

    DF = st.downforce_total
    drag = st.drag
    LD = st.L_over_D
    bal = st.front_balance
    # Stability: prefer slight rear aero bias (bal < 0.45)
    stability = float(np.exp(-((bal - 0.40) ** 2) / 0.02))

    # Simple lap-time proxy: straights love low drag, corners love DF
    # t ~ k1/sqrt(DF) + k2*drag  (normalized)
    DF_n = max(DF, 100.0)
    drag_n = max(drag, 50.0)
    lap_time = 80.0 * (1.0 / np.sqrt(DF_n / 3000.0)) + 0.015 * drag_n
    # DRS schedule reduces straight time slightly
    lap_time *= 1.0 - 0.04 * design.drs_schedule
    fuel = drag * speed * 0.001

    return Objectives(
        downforce=DF,
        drag=drag,
        L_over_D=LD,
        front_balance=bal,
        stability=stability,
        lap_time=float(lap_time),
        fuel_proxy=float(fuel),
    )
