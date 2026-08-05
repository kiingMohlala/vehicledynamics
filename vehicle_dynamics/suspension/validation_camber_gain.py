"""
Phase 6.4 – Camber gain validation.
"""

from __future__ import annotations

import numpy as np
from .camber_gain import compute_camber_gain, update_camber_state, CamberGainModel
from .camber_state import CamberGainParams, CamberState
from .geometry_state import VehicleGeometryState
from ..dual_track.suspension_interface import (
    SuspensionInterface,
    SuspensionInterfaceConfig,
)


def test_zero_travel_zero_gain() -> tuple[bool, dict]:
    p = CamberGainParams.symmetric(front=0.5, rear=0.3)
    cg = compute_camber_gain(np.zeros(4), p)
    ok = bool(np.allclose(cg, 0.0))
    return ok, {"camber_gain": cg.tolist()}


def test_compression_trend() -> tuple[bool, dict]:
    p = CamberGainParams.symmetric(front=0.4, rear=0.0)
    z = np.array([0.05, 0.05, 0.0, 0.0])
    cg = compute_camber_gain(z, p)
    ok = cg[0] > 0 and cg[1] > 0 and abs(cg[0] - 0.02) < 1e-12
    return ok, {"camber_gain": cg.tolist()}


def test_rebound_opposite_sign() -> tuple[bool, dict]:
    p = CamberGainParams.symmetric(front=0.4)
    up = compute_camber_gain(np.array([0.03, 0, 0, 0]), p)
    dn = compute_camber_gain(np.array([-0.03, 0, 0, 0]), p)
    ok = up[0] > 0 and dn[0] < 0 and abs(up[0] + dn[0]) < 1e-12
    return ok, {"up": float(up[0]), "down": float(dn[0])}


def test_left_right_symmetry() -> tuple[bool, dict]:
    p = CamberGainParams.symmetric(front=0.25, rear=0.1)
    z = np.array([0.04, 0.04, 0.02, 0.02])
    cg = compute_camber_gain(z, p)
    ok = abs(cg[0] - cg[1]) < 1e-15 and abs(cg[2] - cg[3]) < 1e-15
    return ok, {"camber_gain": cg.tolist()}


def test_independent_wheel_travel() -> tuple[bool, dict]:
    p = CamberGainParams.symmetric(front=0.5)
    z = np.array([0.04, 0.0, 0.0, 0.0])
    cg = compute_camber_gain(z, p)
    ok = abs(cg[0] - 0.02) < 1e-12 and abs(cg[1]) < 1e-15 and abs(cg[2]) < 1e-15
    return ok, {"camber_gain": cg.tolist()}


def test_neutral_gain_matches_phase63() -> tuple[bool, dict]:
    """gain=0 → same effective_steer and camber_total as Phase 6.3 (static only)."""
    g = VehicleGeometryState.neutral()
    g.fl.camber_rad = 0.02
    g.fr.camber_rad = -0.02

    iface_off = SuspensionInterface(
        SuspensionInterfaceConfig(
            enabled=True, bump_steer_enabled=False, camber_gain_enabled=False
        ),
        geometry=g,
    )
    iface_on = SuspensionInterface(
        SuspensionInterfaceConfig(
            enabled=True, bump_steer_enabled=False, camber_gain_enabled=True
        ),
        geometry=g,
        camber_params=CamberGainParams.neutral(),
    )
    z = np.array([0.05, 0.05, 0.03, 0.03])
    d_off = iface_off.effective_steer(0.1, 0.1, wheel_travel=z)
    d_on = iface_on.effective_steer(0.1, 0.1, wheel_travel=z)
    # steer path unchanged (camber does not affect steer)
    ok_steer = bool(np.allclose(d_off, d_on))

    # camber total with zero gain = static only
    iface_on.set_wheel_travel(z)
    ct = iface_on._last_camber.camber_total
    ok_camber = abs(ct[0] - 0.02) < 1e-12 and abs(ct[1] + 0.02) < 1e-12
    ok = ok_steer and ok_camber
    return ok, {
        "d_off": d_off.tolist(),
        "d_on": d_on.tolist(),
        "camber_total": ct.tolist(),
    }


def test_camber_logged() -> tuple[bool, dict]:
    iface = SuspensionInterface(
        SuspensionInterfaceConfig(
            enabled=True, camber_gain_enabled=True
        ),
        camber_params=CamberGainParams.symmetric(0.3),
    )
    iface.effective_steer(0.0, 0.0, wheel_travel=np.array([0.02, 0.01, 0.0, -0.01]))
    diag = iface.diagnostics()
    ok = (
        "camber_gain_rad" in diag
        and "camber_total_rad" in diag
        and "camber_static_rad" in diag
        and len(diag["camber_gain_rad"]) == 4
        and all(np.isfinite(x) for x in diag["camber_gain_rad"])
    )
    return ok, {"diag_keys": [k for k in diag if "camber" in k]}


def test_total_formula() -> tuple[bool, dict]:
    g = VehicleGeometryState.neutral()
    g.fl.camber_rad = 0.01
    iface = SuspensionInterface(
        SuspensionInterfaceConfig(enabled=True, camber_gain_enabled=True),
        geometry=g,
        camber_params=CamberGainParams(gain_fl=0.5),
    )
    z = np.array([0.04, 0, 0, 0])  # gain_fl * 0.04 = 0.02
    iface.set_wheel_travel(z)
    ct = iface._last_camber.camber_total
    ok = abs(ct[0] - 0.03) < 1e-12  # 0.01 + 0.02
    return ok, {"camber_total_fl": float(ct[0]), "expected": 0.03}


def test_no_nan_inf() -> tuple[bool, dict]:
    p = CamberGainParams.symmetric(front=1.0, rear=-0.5)
    z = np.linspace(-0.1, 0.1, 4)
    st = update_camber_state(z, np.zeros(4), p)
    ok = all(
        np.all(np.isfinite(a))
        for a in (st.camber_gain, st.camber_total, st.wheel_travel)
    )
    return ok, {"camber_gain": st.camber_gain.tolist()}


def run_phase64_validation() -> bool:
    print("=== Phase 6.4 Camber Gain Validation ===\n")
    tests = [
        ("zero_travel_zero_gain", test_zero_travel_zero_gain),
        ("compression_trend", test_compression_trend),
        ("rebound_opposite_sign", test_rebound_opposite_sign),
        ("left_right_symmetry", test_left_right_symmetry),
        ("independent_wheel_travel", test_independent_wheel_travel),
        ("neutral_gain_matches_phase63", test_neutral_gain_matches_phase63),
        ("camber_logged", test_camber_logged),
        ("total_formula", test_total_formula),
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
    run_phase64_validation()
