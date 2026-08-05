"""
Phase 6.3 – Bump-steer validation.
"""

from __future__ import annotations

import numpy as np
from .bump_steer import compute_toe_bump, update_bump_state, BumpSteerModel
from .bump_state import BumpSteerParams, BumpSteerState
from .geometry_state import VehicleGeometryState
from ..dual_track.suspension_interface import (
    SuspensionInterface,
    SuspensionInterfaceConfig,
)


def test_zero_travel_zero_bump() -> tuple[bool, dict]:
    p = BumpSteerParams.symmetric(front=0.5, rear=0.3)
    tb = compute_toe_bump(np.zeros(4), p)
    ok = bool(np.allclose(tb, 0.0))
    return ok, {"toe_bump": tb.tolist()}


def test_positive_bump_trend() -> tuple[bool, dict]:
    p = BumpSteerParams.symmetric(front=0.4, rear=0.0)
    z = np.array([0.05, 0.05, 0.0, 0.0])
    tb = compute_toe_bump(z, p)
    ok = tb[0] > 0 and tb[1] > 0 and abs(tb[0] - 0.02) < 1e-12
    return ok, {"toe_bump": tb.tolist()}


def test_rebound_opposite_sign() -> tuple[bool, dict]:
    p = BumpSteerParams.symmetric(front=0.4)
    tb_up = compute_toe_bump(np.array([0.03, 0, 0, 0]), p)
    tb_dn = compute_toe_bump(np.array([-0.03, 0, 0, 0]), p)
    ok = tb_up[0] > 0 and tb_dn[0] < 0 and abs(tb_up[0] + tb_dn[0]) < 1e-12
    return ok, {"up": float(tb_up[0]), "down": float(tb_dn[0])}


def test_left_right_symmetry() -> tuple[bool, dict]:
    p = BumpSteerParams.symmetric(front=0.25, rear=0.1)
    z = np.array([0.04, 0.04, 0.02, 0.02])
    tb = compute_toe_bump(z, p)
    ok = abs(tb[0] - tb[1]) < 1e-15 and abs(tb[2] - tb[3]) < 1e-15
    return ok, {"toe_bump": tb.tolist()}


def test_independent_wheel_bump() -> tuple[bool, dict]:
    p = BumpSteerParams.symmetric(front=0.5)
    z = np.array([0.04, 0.0, 0.0, 0.0])
    tb = compute_toe_bump(z, p)
    ok = abs(tb[0] - 0.02) < 1e-12 and abs(tb[1]) < 1e-15 and abs(tb[2]) < 1e-15
    return ok, {"toe_bump": tb.tolist()}


def test_neutral_gain_matches_phase62() -> tuple[bool, dict]:
    """gain=0 → same effective_steer as Phase 6.2 (static toe only)."""
    g = VehicleGeometryState.neutral()
    g.fl.toe_rad = 0.01
    g.fr.toe_rad = 0.01

    # Phase 6.2 path: bump disabled
    iface_62 = SuspensionInterface(
        SuspensionInterfaceConfig(enabled=True, bump_steer_enabled=False),
        geometry=g,
    )
    # Phase 6.3 with zero gains
    iface_63 = SuspensionInterface(
        SuspensionInterfaceConfig(enabled=True, bump_steer_enabled=True),
        geometry=g,
        bump_params=BumpSteerParams.neutral(),
    )
    z = np.array([0.05, 0.05, 0.03, 0.03])
    d62 = iface_62.effective_steer(0.1, 0.1, wheel_travel=z)
    d63 = iface_63.effective_steer(0.1, 0.1, wheel_travel=z)
    ok = bool(np.allclose(d62, d63))
    return ok, {"d62": d62.tolist(), "d63": d63.tolist()}


def test_integration_formula() -> tuple[bool, dict]:
    g = VehicleGeometryState.neutral()
    g.fl.toe_rad = 0.01  # static
    iface = SuspensionInterface(
        SuspensionInterfaceConfig(enabled=True, bump_steer_enabled=True),
        geometry=g,
        bump_params=BumpSteerParams(gain_fl=0.5),
    )
    z = np.array([0.04, 0, 0, 0])  # toe_bump_fl = 0.02
    d = iface.effective_steer(0.05, 0.0, wheel_travel=z)
    # δ_eff_fl = 0.05 + 0.01 + 0.02 = 0.08
    ok = abs(d[0] - 0.08) < 1e-12
    return ok, {"delta_eff_fl": float(d[0]), "expected": 0.08}


def test_diagnostics_logged() -> tuple[bool, dict]:
    iface = SuspensionInterface(
        SuspensionInterfaceConfig(enabled=True, bump_steer_enabled=True),
        bump_params=BumpSteerParams.symmetric(0.3),
    )
    iface.effective_steer(0.0, 0.0, wheel_travel=np.array([0.02, 0.01, 0.0, -0.01]))
    diag = iface.diagnostics()
    ok = (
        "toe_bump_rad" in diag
        and "wheel_travel_m" in diag
        and "toe_total_rad" in diag
        and len(diag["toe_bump_rad"]) == 4
        and all(np.isfinite(x) for x in diag["toe_bump_rad"])
    )
    return ok, {"diag_keys": list(diag.keys())}


def test_no_nan_inf() -> tuple[bool, dict]:
    p = BumpSteerParams.symmetric(front=1.0, rear=-0.5)
    z = np.linspace(-0.1, 0.1, 4)
    st = update_bump_state(z, np.zeros(4), p)
    ok = all(
        np.all(np.isfinite(a))
        for a in (st.toe_bump, st.toe_total, st.wheel_travel)
    )
    return ok, {"toe_bump": st.toe_bump.tolist()}


def run_phase63_validation() -> bool:
    print("=== Phase 6.3 Bump Steer Validation ===\n")
    tests = [
        ("zero_travel_zero_bump", test_zero_travel_zero_bump),
        ("positive_bump_trend", test_positive_bump_trend),
        ("rebound_opposite_sign", test_rebound_opposite_sign),
        ("left_right_symmetry", test_left_right_symmetry),
        ("independent_wheel_bump", test_independent_wheel_bump),
        ("neutral_gain_matches_phase62", test_neutral_gain_matches_phase62),
        ("integration_formula", test_integration_formula),
        ("diagnostics_logged", test_diagnostics_logged),
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
    run_phase63_validation()
