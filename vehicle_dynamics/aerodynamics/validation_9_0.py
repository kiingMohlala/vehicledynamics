"""Phase 9.0 – Aerodynamics foundation validation (target 10/10)."""

from __future__ import annotations

import numpy as np

from .coefficients import AeroConfig, AeroCoefficients
from .ride_height import RideHeightState, ride_height_factors
from .aero_model import compute_aero_loads
from .aero_solver import solve_aero
from .aero_map import build_default_map


def test_zero_speed() -> tuple[bool, dict]:
    cfg = AeroConfig()
    st = compute_aero_loads(0.0, cfg)
    ok = (
        st.Fx == 0
        and st.Fy == 0
        and st.Fz_front == 0
        and st.Fz_rear == 0
        and st.q == 0
    )
    return ok, {"q": st.q, "drag": st.drag}


def test_drag_v2() -> tuple[bool, dict]:
    cfg = AeroConfig()
    s1 = compute_aero_loads(20.0, cfg)
    s2 = compute_aero_loads(40.0, cfg)
    ratio = s2.drag / max(s1.drag, 1e-12)
    ok = abs(ratio - 4.0) < 0.05
    return ok, {"ratio": ratio, "expected": 4.0}


def test_downforce_v2() -> tuple[bool, dict]:
    cfg = AeroConfig()
    s1 = compute_aero_loads(20.0, cfg)
    s2 = compute_aero_loads(40.0, cfg)
    ratio = s2.downforce_total / max(s1.downforce_total, 1e-12)
    ok = abs(ratio - 4.0) < 0.05
    return ok, {"ratio": ratio}


def test_front_rear_balance() -> tuple[bool, dict]:
    cfg = AeroConfig()
    st = compute_aero_loads(50.0, cfg)
    bal = st.front_balance
    # Cl_front=-0.45, Cl_rear=-0.70 → front share ≈ 0.45/1.15 ≈ 0.391
    expected = abs(cfg.coeffs.Cl_front) / (
        abs(cfg.coeffs.Cl_front) + abs(cfg.coeffs.Cl_rear)
    )
    ok = abs(bal - expected) < 0.02
    return ok, {"balance": bal, "expected": expected}


def test_ride_height_sensitivity() -> tuple[bool, dict]:
    cfg = AeroConfig()
    high = RideHeightState(h_front=0.12, h_rear=0.14)
    low = RideHeightState(h_front=0.05, h_rear=0.06)
    s_high = compute_aero_loads(50.0, cfg, ride=high)
    s_low = compute_aero_loads(50.0, cfg, ride=low)
    ok = s_low.downforce_total > s_high.downforce_total * 1.05
    return ok, {
        "DF_low": s_low.downforce_total,
        "DF_high": s_high.downforce_total,
    }


def test_center_of_pressure() -> tuple[bool, dict]:
    cfg = AeroConfig()
    # More rear downforce → CoP aft of mid (positive x_cp with our sign)
    st = compute_aero_loads(50.0, cfg)
    ok = st.center_of_pressure_x > 0  # rear-biased
    # Pitch nose-up should move CoP further aft
    pitched = RideHeightState(
        h_front=cfg.h_front_ref, h_rear=cfg.h_rear_ref, pitch_rad=0.02
    )
    st2 = compute_aero_loads(50.0, cfg, ride=pitched)
    ok = ok and st2.center_of_pressure_x >= st.center_of_pressure_x - 1e-6
    return ok, {
        "x_cp_mm": st.center_of_pressure_x * 1e3,
        "x_cp_pitch_mm": st2.center_of_pressure_x * 1e3,
    }


def test_tire_load_coupling() -> tuple[bool, dict]:
    res = solve_aero(50.0)
    ok = res.dFz_front > 0 and res.dFz_rear > 0
    ok = ok and abs(res.dFz_front - res.state.downforce_front) < 1e-6
    ok = ok and abs(res.dFz_rear - res.state.downforce_rear) < 1e-6
    return ok, {"dFz_f": res.dFz_front, "dFz_r": res.dFz_rear}


def test_handling_integration() -> tuple[bool, dict]:
    """Aero map + solver produce finite loads usable by handling metrics."""
    cfg = AeroConfig()
    amap = build_default_map(cfg)
    res = solve_aero(60.0, cfg=cfg, aero_map=amap)
    ok = (
        res.state.L_over_D > 0
        and res.drag_power > 0
        and np.isfinite(res.state.My)
        and 0.0 < res.state.front_balance < 1.0
    )
    return ok, {"L/D": res.state.L_over_D, "power_kW": res.drag_power / 1e3}


def test_regression_disabled() -> tuple[bool, dict]:
    cfg = AeroConfig(enabled=False)
    res = solve_aero(80.0, cfg=cfg)
    ok = (
        res.state.drag == 0
        and res.dFz_front == 0
        and res.dFz_rear == 0
        and res.drag_power == 0
    )
    return ok, {"enabled": cfg.enabled, "drag": res.state.drag}


def test_no_nan_inf() -> tuple[bool, dict]:
    cfg = AeroConfig()
    rides = [
        RideHeightState(0.08, 0.10, 0.0, 0.0),
        RideHeightState(0.03, 0.03, 0.05, 0.1),
        RideHeightState(0.15, 0.15, -0.03, -0.2),
    ]
    ok = True
    for r in rides:
        for v in (0.0, 15.0, 55.0):
            st = compute_aero_loads(v, cfg, ride=r)
            vals = [
                st.Fx, st.Fy, st.Fz_front, st.Fz_rear,
                st.My, st.Mz, st.q, st.L_over_D,
            ]
            if not all(np.isfinite(vals)):
                ok = False
    return ok, {"ok": ok}


def run_phase90_validation() -> bool:
    print("=== Phase 9.0 Aerodynamics Foundation Validation ===\n")
    tests = [
        ("zero_speed", test_zero_speed),
        ("drag_proportional_v2", test_drag_v2),
        ("downforce_proportional_v2", test_downforce_v2),
        ("front_rear_balance", test_front_rear_balance),
        ("ride_height_sensitivity", test_ride_height_sensitivity),
        ("center_of_pressure", test_center_of_pressure),
        ("tire_load_coupling", test_tire_load_coupling),
        ("handling_integration", test_handling_integration),
        ("regression_disabled", test_regression_disabled),
        ("no_nan_inf", test_no_nan_inf),
    ]
    all_pass = True
    for name, fn in tests:
        try:
            ok, diag = fn()
        except Exception as e:
            ok, diag = False, {"error": str(e)}
        print(f"{name:36} : {'PASS' if ok else 'FAIL'}")
        for k, v in list(diag.items())[:6]:
            print(f"    {k}: {v}")
        if not ok:
            all_pass = False
    print("\n=========================================")
    print("ALL TESTS PASSED" if all_pass else "SOME FAILED")
    if all_pass:
        print("Phase 9.0 Status: VALIDATED ✓")
    return all_pass


if __name__ == "__main__":
    run_phase90_validation()
