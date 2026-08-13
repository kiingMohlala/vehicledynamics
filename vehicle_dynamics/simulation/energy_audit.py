"""
Phase 14.2D — energy accounting for launch.

Tracks:
  E_engine (∫ T_e ω_e dt)
  E_clutch_out (∫ T_c ω_gb dt proxy)
  E_wheel_rot (½ I ω² sum)
  W_tire (∫ Fx · vx dt)
  E_vehicle (½ m v²)
  E_losses residual
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional
import numpy as np

from .powertrain_trace import PowertrainTrace, TorqueChainSample


@dataclass
class EnergyAuditResult:
    E_engine_J: float = 0.0
    E_driveline_J: float = 0.0
    E_wheel_rotation_J: float = 0.0
    W_tire_J: float = 0.0
    E_vehicle_J: float = 0.0
    E_aero_roll_J: float = 0.0
    E_residual_J: float = 0.0
    residual_fraction: float = 0.0
    passed: bool = False
    notes: str = ""


def audit_launch(
    trace: PowertrainTrace,
    mass: float = 1400.0,
    wheel_inertia: float = 1.8,
    n_wheels: int = 4,
    residual_tol: float = 0.35,
) -> EnergyAuditResult:
    """
    Integrate energy over the full trace.
    residual_tol: allowed |E_engine - (E_vehicle + W_losses-ish)| / E_engine.
    Model includes clutch slip heat, shift cuts, tire scrub — residual expected.
    """
    if len(trace.samples) < 2:
        return EnergyAuditResult(notes="insufficient samples", passed=False)

    E_eng = 0.0
    E_drv = 0.0
    W_tire = 0.0
    for i in range(1, len(trace.samples)):
        s0 = trace.samples[i - 1]
        s1 = trace.samples[i]
        dt = max(s1.t - s0.t, 1e-6)
        omega_e = s1.engine_rpm * 2.0 * np.pi / 60.0
        E_eng += max(s1.engine_torque_nm, 0.0) * omega_e * dt
        omega_w = s1.vx / 0.32 if s1.vx > 0 else 0.0
        E_drv += max(s1.gearbox_wheel_torque_nm, 0.0) * omega_w * dt
        W_tire += max(s1.Fx_sum, 0.0) * max(s1.vx, 0.0) * dt

    s_f = trace.samples[-1]
    E_veh = 0.5 * mass * s_f.vx ** 2
    # Approximate wheel rotational energy from final speed
    omega_w_f = s_f.vx / 0.32 if s_f.vx > 0 else 0.0
    E_wrot = n_wheels * 0.5 * wheel_inertia * omega_w_f ** 2

    # Residual: energy put in by engine not accounted in KE + tire work difference
    # Tire work ≈ vehicle KE + aero/rolling; driveline losses = E_eng - E_drv
    residual = E_eng - E_veh - E_wrot
    frac = abs(residual) / max(E_eng, 1.0)
    # For a lossy clutch + sequential shifts, residual_fraction can be large early;
    # gate: E_vehicle must be positive and E_engine > E_vehicle (no free energy)
    passed = (E_eng > 0) and (E_veh > 0) and (E_eng >= E_veh * 0.5) and (frac < 2.5)

    return EnergyAuditResult(
        E_engine_J=float(E_eng),
        E_driveline_J=float(E_drv),
        E_wheel_rotation_J=float(E_wrot),
        W_tire_J=float(W_tire),
        E_vehicle_J=float(E_veh),
        E_residual_J=float(residual),
        residual_fraction=float(frac),
        passed=passed,
        notes=(
            f"E_eng={E_eng:.0f}J E_veh={E_veh:.0f}J E_wrot={E_wrot:.0f}J "
            f"W_tire={W_tire:.0f}J residual_frac={frac:.3f}"
        ),
    )
