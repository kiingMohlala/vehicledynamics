"""
Phase 6.2 – Suspension geometry coupling validation.

Includes neutral-geometry regression against the Phase 5 / v1.0 baseline contract:
  toe=0, camber=0, IR=1  →  identical steering path to legacy dual-track.
"""

from __future__ import annotations

import numpy as np

from .geometry_state import VehicleGeometryState, WheelGeometryState
from .coupling import CoupledSuspension
from .wheel_rate import SpringDamperParams, MotionRatioParams, compute_wheel_rate
from ..dual_track.suspension_interface import (
    SuspensionInterface,
    SuspensionInterfaceConfig,
)


def test_zero_geometry_offsets() -> tuple[bool, dict]:
    g = VehicleGeometryState.neutral()
    ok = (
        abs(g.fl.toe_rad) < 1e-15
        and abs(g.fl.camber_rad) < 1e-15
        and abs(g.fl.installation_ratio - 1.0) < 1e-15
        and abs(g.fl.motion_ratio - 1.0) < 1e-15
    )
    return ok, {"toe": g.fl.toe_rad, "camber": g.fl.camber_rad, "IR": g.fl.installation_ratio}


def test_toe_changes_heading_only() -> tuple[bool, dict]:
    g = VehicleGeometryState.neutral()
    # inject toe on FL only
    g.fl.toe_rad = 0.02
    iface = SuspensionInterface(
        SuspensionInterfaceConfig(enabled=True),
        geometry=g,
    )
    d = iface.effective_steer(0.10, 0.10, 0.0, 0.0)
    ok = abs(d[0] - 0.12) < 1e-12 and abs(d[1] - 0.10) < 1e-12
    # disabled → no toe
    iface_off = SuspensionInterface(SuspensionInterfaceConfig(enabled=False), geometry=g)
    d_off = iface_off.effective_steer(0.10, 0.10, 0.0, 0.0)
    ok = ok and abs(d_off[0] - 0.10) < 1e-12
    return ok, {"d_enabled": d.tolist(), "d_disabled": d_off.tolist()}


def test_camber_logged_not_forced() -> tuple[bool, dict]:
    """Camber present in diagnostics; no tire force API in interface."""
    g = VehicleGeometryState.neutral()
    g.fl.camber_rad = 0.05
    iface = SuspensionInterface(SuspensionInterfaceConfig(enabled=True), geometry=g)
    diag = iface.diagnostics()
    ok = abs(diag["camber_rad"][0] - 0.05) < 1e-12
    ok = ok and not hasattr(iface, "camber_force")  # must not generate forces
    return ok, {"camber_diag": diag["camber_rad"]}


def test_wheel_rate_from_mr() -> tuple[bool, dict]:
    wr = compute_wheel_rate(
        SpringDamperParams(Ks=30000, Cs=2000),
        MotionRatioParams(installation_ratio=0.8, layout="pushrod"),
    )
    g = VehicleGeometryState.neutral()
    g.fl.Kw = wr.Kw
    g.fl.Cw = wr.Cw
    g.fl.installation_ratio = wr.installation_ratio
    iface = SuspensionInterface(SuspensionInterfaceConfig(enabled=True), geometry=g)
    F = iface.vertical_forces(np.array([0.01, 0, 0, 0]), np.zeros(4))
    ok = abs(F[0] - wr.Kw * 0.01) < 1e-9 and abs(F[1]) < 1e-15
    return ok, {"F0": F[0], "Kw": wr.Kw}


def test_left_right_symmetry() -> tuple[bool, dict]:
    susp = CoupledSuspension()
    states = susp.evaluate_all()
    ok = (
        abs(states["FL"].Kw - states["FR"].Kw) < 1e-9
        and abs(states["FL"].camber_rad + states["FR"].camber_rad) < 1e-5
    )
    return ok, {
        "Kw_FL": states["FL"].Kw,
        "Kw_FR": states["FR"].Kw,
        "camber_FL": states["FL"].camber_deg,
        "camber_FR": states["FR"].camber_deg,
    }


def test_neutral_matches_baseline_steer() -> tuple[bool, dict]:
    """
    Neutral geometry + interface enabled must give same δ as disabled
    (Phase 5 path): toe=0 → δ_eff = δ_cmd.
    """
    g = VehicleGeometryState.neutral()
    on = SuspensionInterface(SuspensionInterfaceConfig(enabled=True), geometry=g)
    off = SuspensionInterface(SuspensionInterfaceConfig(enabled=False), geometry=g)
    for dfl, dfr in [(0.0, 0.0), (0.05, 0.05), (-0.03, 0.04)]:
        a = on.effective_steer(dfl, dfr)
        b = off.effective_steer(dfl, dfr)
        if not np.allclose(a, b):
            return False, {"on": a.tolist(), "off": b.tolist(), "cmd": (dfl, dfr)}
    return True, {"note": "neutral toe → identical to Phase 5 steer path"}


def test_kpi_caster_rc_logged() -> tuple[bool, dict]:
    susp = CoupledSuspension()
    states = susp.evaluate_all()
    ok = all(
        np.isfinite(states[k].kpi_rad)
        and np.isfinite(states[k].caster_rad)
        and np.isfinite(states[k].roll_center_z)
        for k in ("FL", "FR", "RL", "RR")
    )
    return ok, {
        "kpi_FL_deg": float(np.degrees(states["FL"].kpi_rad)),
        "caster_FL_deg": float(np.degrees(states["FL"].caster_rad)),
        "rc_z_FL": states["FL"].roll_center_z,
    }


def test_no_nan_inf() -> tuple[bool, dict]:
    iface = SuspensionInterface(
        SuspensionInterfaceConfig(enabled=True, use_geometry_solver=True)
    )
    d = iface.effective_steer(0.1, 0.1)
    F = iface.vertical_forces(np.ones(4) * 0.01, np.zeros(4))
    diag = iface.diagnostics()
    flat = list(d) + list(F)
    for v in diag["camber_rad"] + diag["toe_rad"] + diag["Kw"]:
        flat.append(v)
    ok = all(np.isfinite(x) for x in flat)
    return ok, {"n_values": len(flat)}


def run_phase62_validation() -> bool:
    print("=== Phase 6.2 Suspension Geometry Coupling Validation ===\n")
    tests = [
        ("zero_geometry_offsets", test_zero_geometry_offsets),
        ("toe_changes_heading_only", test_toe_changes_heading_only),
        ("camber_logged_not_forced", test_camber_logged_not_forced),
        ("wheel_rate_from_mr", test_wheel_rate_from_mr),
        ("left_right_symmetry", test_left_right_symmetry),
        ("neutral_matches_baseline_steer", test_neutral_matches_baseline_steer),
        ("kpi_caster_rc_logged", test_kpi_caster_rc_logged),
        ("no_nan_inf", test_no_nan_inf),
    ]
    all_pass = True
    for name, fn in tests:
        try:
            ok, diag = fn()
        except Exception as e:
            ok, diag = False, {"error": str(e)}
        print(f"{name:36} : {'PASS' if ok else 'FAIL'}")
        for k, v in diag.items():
            print(f"    {k}: {v}")
        if not ok:
            all_pass = False
    print("\nOverall:", "ALL PASSED" if all_pass else "SOME FAILED")
    return all_pass


if __name__ == "__main__":
    run_phase62_validation()
