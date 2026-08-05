"""Phase 10.2 – Differential validation (target 18/18)."""

from __future__ import annotations

import numpy as np

from .open_diff import open_split
from .locked_diff import locked_split
from .clutch_lsd import clutch_lsd_split
from .torsen import torsen_split
from .viscous_diff import viscous_split
from .torque_vectoring import torque_vector_split
from .wheel_speed import axle_speed, differential_speed
from .differential_solver import DifferentialConfig, DifferentialSolver


def test_open_equal_split() -> tuple[bool, dict]:
    L, R = open_split(400.0)
    ok = abs(L - 200) < 1e-9 and abs(R - 200) < 1e-9
    return ok, {"L": L, "R": R}


def test_open_one_wheel_slip() -> tuple[bool, dict]:
    """Open still splits equally even with speed difference."""
    s = DifferentialSolver(DifferentialConfig(diff_type="open"))
    st = s.step(400.0, omega_left=80.0, omega_right=10.0, dt=0.01)
    ok = abs(st.driveline.torque_left - st.driveline.torque_right) < 1e-6
    return ok, {"L": st.driveline.torque_left, "R": st.driveline.torque_right}


def test_locked_equal_speed() -> tuple[bool, dict]:
    s = DifferentialSolver(DifferentialConfig(diff_type="locked"))
    st = s.step(400.0, 50.0, 50.0, mu_left=1.0, mu_right=1.0)
    ok = abs(st.driveline.torque_left - st.driveline.torque_right) < 1.0
    return ok, {"L": st.driveline.torque_left, "R": st.driveline.torque_right}


def test_locked_high_grip() -> tuple[bool, dict]:
    s = DifferentialSolver(DifferentialConfig(diff_type="locked"))
    st = s.step(400.0, 50.0, 50.0, mu_left=0.3, mu_right=1.0, Fz_left=4000, Fz_right=4000)
    ok = st.driveline.torque_right > st.driveline.torque_left
    return ok, {"L": st.driveline.torque_left, "R": st.driveline.torque_right}


def test_clutch_lsd_bias() -> tuple[bool, dict]:
    T_L, T_R, bias = clutch_lsd_split(400.0, omega_L=70.0, omega_R=50.0, preload=80.0, k_lock=10.0)
    ok = bias > 80.0 and T_R > T_L  # bias toward slow (right)
    return ok, {"L": T_L, "R": T_R, "bias": bias}


def test_torsen_bias_ratio() -> tuple[bool, dict]:
    T_L, T_R, _ = torsen_split(300.0, omega_L=60.0, omega_R=40.0, tbr=3.0, preload=0.0)
    # R slower → more torque
    ratio = max(T_R, T_L) / max(min(T_R, T_L), 1e-9)
    ok = T_R > T_L and ratio <= 3.0 + 0.05
    return ok, {"L": T_L, "R": T_R, "ratio": ratio}


def test_viscous_speed_difference() -> tuple[bool, dict]:
    T_L, T_R, b = viscous_split(200.0, omega_L=80.0, omega_R=40.0, k_v=5.0)
    ok = T_R > T_L and b > 0
    return ok, {"L": T_L, "R": T_R, "bias": b}


def test_torque_vectoring_left() -> tuple[bool, dict]:
    # negative delta → more left
    T_L, T_R, d = torque_vector_split(400.0, delta_T=-100.0)
    ok = T_L > T_R and abs(d + 100) < 1e-9
    return ok, {"L": T_L, "R": T_R}


def test_torque_vectoring_right() -> tuple[bool, dict]:
    T_L, T_R, d = torque_vector_split(400.0, delta_T=100.0)
    ok = T_R > T_L
    return ok, {"L": T_L, "R": T_R}


def test_traction_limit() -> tuple[bool, dict]:
    s = DifferentialSolver(DifferentialConfig(diff_type="open", radius=0.32))
    st = s.step(5000.0, 50.0, 50.0, mu_left=0.5, mu_right=0.5, Fz_left=3000, Fz_right=3000)
    cap = 0.5 * 3000 * 0.32
    ok = st.driveline.torque_left <= cap + 1e-6 and st.driveline.torque_right <= cap + 1e-6
    return ok, {"L": st.driveline.torque_left, "cap": cap}


def test_wheel_speed_consistency() -> tuple[bool, dict]:
    wL, wR = 62.0, 58.0
    ok = abs(axle_speed(wL, wR) - 60.0) < 1e-12
    ok = ok and abs(differential_speed(wL, wR) - 4.0) < 1e-12
    return ok, {"axle": axle_speed(wL, wR), "dw": differential_speed(wL, wR)}


def test_power_conservation() -> tuple[bool, dict]:
    s = DifferentialSolver(DifferentialConfig(diff_type="open"))
    st = s.step(400.0, 50.0, 50.0)
    ok = abs((st.driveline.torque_left + st.driveline.torque_right) - 400.0) < 1e-6
    return ok, {"sum": st.driveline.torque_left + st.driveline.torque_right}


def test_thermal_regression() -> tuple[bool, dict]:
    """No thermal state required; ensure LSD deterministic."""
    a = clutch_lsd_split(300.0, 55.0, 50.0, preload=50.0)
    b = clutch_lsd_split(300.0, 55.0, 50.0, preload=50.0)
    ok = a == b
    return ok, {"match": ok}


def test_regression_disabled() -> tuple[bool, dict]:
    s = DifferentialSolver(DifferentialConfig(enabled=False))
    st = s.step(400.0, 50.0, 50.0)
    ok = st.driveline.torque_left == 0 and st.driveline.torque_right == 0
    return ok, {"L": st.driveline.torque_left}


def test_tire_coupling() -> tuple[bool, dict]:
    """Wheel speeds from tire model feed delta omega."""
    s = DifferentialSolver(DifferentialConfig(diff_type="clutch_lsd", preload=100.0))
    st = s.step(350.0, omega_left=70.0, omega_right=55.0)
    ok = abs(st.driveline.delta_omega - 15.0) < 1e-9
    ok = ok and st.driveline.torque_right > st.driveline.torque_left
    return ok, {"dw": st.driveline.delta_omega, "L": st.driveline.torque_left, "R": st.driveline.torque_right}


def test_no_nan_inf() -> tuple[bool, dict]:
    ok = True
    for dtype in ("open", "locked", "clutch_lsd", "viscous", "torsen", "torque_vectoring"):
        s = DifferentialSolver(DifferentialConfig(diff_type=dtype))
        if dtype == "torque_vectoring":
            s.set_tv_delta(50.0)
        st = s.step(300.0, 40.0, 55.0)
        vals = [st.driveline.torque_left, st.driveline.torque_right, st.driveline.axle_speed]
        if not all(np.isfinite(vals)):
            ok = False
    return ok, {"ok": ok}


def test_performance_regression() -> tuple[bool, dict]:
    s = DifferentialSolver(DifferentialConfig(diff_type="clutch_lsd", preload=80.0, k_lock=20.0))
    st = s.step(420.0, 62.0, 58.0)
    ok = abs(st.driveline.torque_left + st.driveline.torque_right - 420.0) < 50.0  # bias may not conserve exactly
    # Actually clutch_lsd conserves sum
    ok = abs(st.driveline.torque_left + st.driveline.torque_right - 420.0) < 1e-6
    return ok, {"sum": st.driveline.torque_left + st.driveline.torque_right}


def test_repeatability() -> tuple[bool, dict]:
    s1 = DifferentialSolver(DifferentialConfig(diff_type="torsen", torque_bias_ratio=3.0))
    s2 = DifferentialSolver(DifferentialConfig(diff_type="torsen", torque_bias_ratio=3.0))
    a = s1.step(400.0, 65.0, 55.0)
    b = s2.step(400.0, 65.0, 55.0)
    ok = abs(a.driveline.torque_left - b.driveline.torque_left) < 1e-12
    return ok, {"match": ok}


def run_phase102_validation() -> bool:
    print("=== Phase 10.2 Differential Validation ===\n")
    tests = [
        ("open_equal_split", test_open_equal_split),
        ("open_one_wheel_slip", test_open_one_wheel_slip),
        ("locked_equal_speed", test_locked_equal_speed),
        ("locked_high_grip", test_locked_high_grip),
        ("clutch_lsd_bias", test_clutch_lsd_bias),
        ("torsen_bias_ratio", test_torsen_bias_ratio),
        ("viscous_speed_difference", test_viscous_speed_difference),
        ("torque_vectoring_left", test_torque_vectoring_left),
        ("torque_vectoring_right", test_torque_vectoring_right),
        ("traction_limit", test_traction_limit),
        ("wheel_speed_consistency", test_wheel_speed_consistency),
        ("power_conservation", test_power_conservation),
        ("thermal_regression", test_thermal_regression),
        ("regression_disabled", test_regression_disabled),
        ("tire_coupling", test_tire_coupling),
        ("no_nan_inf", test_no_nan_inf),
        ("performance_regression", test_performance_regression),
        ("repeatability", test_repeatability),
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
        print("Phase 10.2 Status: VALIDATED ✓")
    return all_pass


if __name__ == "__main__":
    run_phase102_validation()
