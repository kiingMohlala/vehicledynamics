"""Fit analytical AeroConfig coefficients to CFD/WT samples."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from vehicle_dynamics.aerodynamics.coefficients import AeroConfig, AeroCoefficients
from vehicle_dynamics.aerodynamics.ride_height import RideHeightState
from vehicle_dynamics.aerodynamics.aero_model import compute_aero_loads
from .cfd_map import AeroSample


@dataclass
class CalibrationResult:
    config: AeroConfig
    rms_Cd: float
    rms_Cl_f: float
    rms_Cl_r: float
    max_Cd: float
    max_Cl_f: float
    max_Cl_r: float
    mean_bias_Cd: float
    n_samples: int
    success: bool = True
    message: str = "ok"


def calibrate_against_samples(
    samples: list[AeroSample],
    base: AeroConfig | None = None,
) -> CalibrationResult:
    """
    Least-squares scale factors on Cd, Cl_front, Cl_rear at reference attitude.

    Uses samples near design ride height when available.
    """
    cfg = base or AeroConfig()
    if not samples:
        return CalibrationResult(
            config=cfg,
            rms_Cd=0, rms_Cl_f=0, rms_Cl_r=0,
            max_Cd=0, max_Cl_f=0, max_Cl_r=0,
            mean_bias_Cd=0, n_samples=0, success=False, message="no samples",
        )

    # Analytical prediction at each sample state
    Cd_a, Clf_a, Clr_a = [], [], []
    Cd_m, Clf_m, Clr_m = [], [], []
    for s in samples:
        ride = RideHeightState(
            h_front=s.h_front, h_rear=s.h_rear, pitch_rad=s.pitch, yaw_rad=s.yaw
        )
        st = compute_aero_loads(s.speed, cfg, ride=ride)
        # Recover effective coeffs from forces
        qS = st.q * cfg.frontal_area
        if qS < 1e-9:
            continue
        Cd_a.append(-st.Fx / qS)
        Clf_a.append(st.Fz_front / qS)
        Clr_a.append(st.Fz_rear / qS)
        Cd_m.append(s.Cd)
        Clf_m.append(s.Cl_front)
        Clr_m.append(s.Cl_rear)

    Cd_a = np.asarray(Cd_a)
    Clf_a = np.asarray(Clf_a)
    Clr_a = np.asarray(Clr_a)
    Cd_m = np.asarray(Cd_m)
    Clf_m = np.asarray(Clf_m)
    Clr_m = np.asarray(Clr_m)

    def scale(meas, anal):
        denom = float(np.dot(anal, anal)) + 1e-15
        return float(np.dot(meas, anal) / denom)

    s_cd = scale(Cd_m, Cd_a)
    s_clf = scale(Clf_m, Clf_a)
    s_clr = scale(Clr_m, Clr_a)

    new_coeffs = AeroCoefficients(
        Cd=cfg.coeffs.Cd * s_cd,
        Cl_front=cfg.coeffs.Cl_front * s_clf,
        Cl_rear=cfg.coeffs.Cl_rear * s_clr,
        Cy_beta=cfg.coeffs.Cy_beta,
        Cm_pitch=cfg.coeffs.Cm_pitch,
        Cn_yaw=cfg.coeffs.Cn_yaw,
    )
    new_cfg = AeroConfig(
        enabled=cfg.enabled,
        rho=cfg.rho,
        frontal_area=cfg.frontal_area,
        wheelbase=cfg.wheelbase,
        track=cfg.track,
        cg_height=cfg.cg_height,
        ref_chord=cfg.ref_chord,
        h_front_ref=cfg.h_front_ref,
        h_rear_ref=cfg.h_rear_ref,
        coeffs=new_coeffs,
        front_wing_share=cfg.front_wing_share,
        rear_wing_share=cfg.rear_wing_share,
        underfloor_share=cfg.underfloor_share,
        body_share=cfg.body_share,
        cooling_drag_fraction=cfg.cooling_drag_fraction,
    )

    # Residuals with calibrated config
    err_cd, err_clf, err_clr = [], [], []
    for s in samples:
        ride = RideHeightState(
            h_front=s.h_front, h_rear=s.h_rear, pitch_rad=s.pitch, yaw_rad=s.yaw
        )
        st = compute_aero_loads(s.speed, new_cfg, ride=ride)
        qS = st.q * new_cfg.frontal_area
        if qS < 1e-9:
            continue
        err_cd.append((-st.Fx / qS) - s.Cd)
        err_clf.append((st.Fz_front / qS) - s.Cl_front)
        err_clr.append((st.Fz_rear / qS) - s.Cl_rear)

    err_cd = np.asarray(err_cd)
    err_clf = np.asarray(err_clf)
    err_clr = np.asarray(err_clr)

    return CalibrationResult(
        config=new_cfg,
        rms_Cd=float(np.sqrt(np.mean(err_cd**2))) if err_cd.size else 0.0,
        rms_Cl_f=float(np.sqrt(np.mean(err_clf**2))) if err_clf.size else 0.0,
        rms_Cl_r=float(np.sqrt(np.mean(err_clr**2))) if err_clr.size else 0.0,
        max_Cd=float(np.max(np.abs(err_cd))) if err_cd.size else 0.0,
        max_Cl_f=float(np.max(np.abs(err_clf))) if err_clf.size else 0.0,
        max_Cl_r=float(np.max(np.abs(err_clr))) if err_clr.size else 0.0,
        mean_bias_Cd=float(np.mean(err_cd)) if err_cd.size else 0.0,
        n_samples=len(samples),
        success=True,
    )
