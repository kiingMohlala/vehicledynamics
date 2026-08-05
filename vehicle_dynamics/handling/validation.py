"""
Phase 7.2 – Handling metrics validation.
"""

from __future__ import annotations

import numpy as np
from . import analyze_run, classify_balance, utilization_metrics
from .metrics import understeer_gradient, G
from .balance import UtilizationMetrics
from .steady_state import extract_steady_state


def _synthetic_circle(
    vx: float = 20.0,
    R: float = 40.0,
    L: float = 2.7,
    K_deg_g: float = 2.0,
    T: float = 5.0,
    dt: float = 0.01,
):
    """Steady circular path with prescribed understeer gradient."""
    t = np.arange(0, T, dt)
    r = vx / R
    ay = vx * r
    # δ = L/R + K*(ay/g)
    delta = L / R + np.radians(K_deg_g) * (ay / G)
    n = len(t)
    vx_a = np.full(n, vx)
    vy_a = np.zeros(n)
    r_a = np.full(n, r)
    delta_a = np.full(n, delta)
    # mild front-biased utilization for understeer
    util = np.column_stack([
        np.full(n, 0.90),
        np.full(n, 0.88),
        np.full(n, 0.75),
        np.full(n, 0.74),
    ])
    return t, vx_a, vy_a, r_a, delta_a, util, L


def test_straight_line_metrics() -> tuple[bool, dict]:
    t = np.linspace(0, 3, 301)
    vx = np.full_like(t, 25.0)
    vy = np.zeros_like(t)
    r = np.zeros_like(t)
    delta = np.zeros_like(t)
    rep = analyze_run(t, vx, vy, r, delta, wheelbase=2.7)
    ok = abs(rep.steady.yaw_rate_ss) < 1e-6
    ok = ok and abs(rep.stability.peak_beta_deg) < 0.1
    ok = ok and np.isfinite(rep.driver.average_speed)
    return ok, {"yaw": rep.steady.yaw_rate_ss, "beta": rep.stability.peak_beta_deg}


def test_constant_radius_corner() -> tuple[bool, dict]:
    t, vx, vy, r, delta, util, L = _synthetic_circle()
    ss = extract_steady_state(t, vx, vy, r, delta, L)
    ok = abs(ss.turning_radius - 40.0) < 0.5
    ok = ok and abs(ss.understeer_gradient_deg_per_g - 2.0) < 0.3
    ok = ok and ss.max_ay_g > 0.5
    return ok, {
        "R": ss.turning_radius,
        "K": ss.understeer_gradient_deg_per_g,
        "ay_g": ss.max_ay_g,
    }


def test_utilization_bounds() -> tuple[bool, dict]:
    u = np.random.uniform(0.2, 0.95, size=(100, 4))
    m = utilization_metrics(u)
    ok = np.all(m.peak <= 1.0 + 1e-9) and np.all(m.mean >= 0.0)
    ok = ok and 0 <= m.limiting_wheel <= 3
    return ok, {"peak": m.peak.tolist(), "axle": m.limiting_axle}


def test_understeer_classification() -> tuple[bool, dict]:
    util = UtilizationMetrics(
        peak=np.array([0.95, 0.93, 0.8, 0.79]),
        mean=np.array([0.9, 0.88, 0.75, 0.74]),
        time_above_90=np.zeros(4),
        limiting_wheel=0,
        limiting_axle="front",
        front_mean=0.89,
        rear_mean=0.745,
    )
    b = classify_balance(2.1, util)
    ok = "Understeer" in b.classification
    return ok, {"class": b.classification}


def test_oversteer_classification() -> tuple[bool, dict]:
    util = UtilizationMetrics(
        peak=np.array([0.7, 0.7, 0.98, 0.97]),
        mean=np.array([0.65, 0.64, 0.92, 0.91]),
        time_above_90=np.zeros(4),
        limiting_wheel=2,
        limiting_axle="rear",
        front_mean=0.645,
        rear_mean=0.915,
    )
    b = classify_balance(-2.0, util)
    ok = "oversteer" in b.classification.lower()
    return ok, {"class": b.classification}


def test_neutral_classification() -> tuple[bool, dict]:
    util = UtilizationMetrics(
        peak=np.array([0.85, 0.85, 0.84, 0.84]),
        mean=np.array([0.8, 0.8, 0.8, 0.8]),
        time_above_90=np.zeros(4),
        limiting_wheel=0,
        limiting_axle="balanced",
        front_mean=0.8,
        rear_mean=0.8,
    )
    b = classify_balance(0.05, util)
    ok = "Neutral" in b.classification
    return ok, {"class": b.classification}


def test_k_formula() -> tuple[bool, dict]:
    # δ = L/R + K_rad * (ay/g)
    L, R, ay = 2.7, 50.0, 4.0
    K_deg = 1.5
    delta = L / R + np.radians(K_deg) * (ay / G)
    K = understeer_gradient(delta, L, R, ay)
    ok = abs(K - K_deg) < 1e-6
    return ok, {"K": K, "expected": K_deg}


def test_no_nan() -> tuple[bool, dict]:
    t, vx, vy, r, delta, util, L = _synthetic_circle()
    rep = analyze_run(t, vx, vy, r, delta, L, utilization=util)
    text = rep.format_text()
    ok = "nan" not in text.lower() and np.isfinite(rep.steady.max_ay)
    return ok, {"lines": len(text.splitlines())}


def test_regression_smoke() -> tuple[bool, dict]:
    """Phase 5.5-style straight + mild steer still produces finite report."""
    t = np.linspace(0, 4, 401)
    vx = np.full_like(t, 22.0)
    vy = 0.02 * np.sin(0.5 * t)
    r = 0.05 * np.sin(0.5 * t)
    delta = 0.03 * np.sin(0.5 * t)
    util = 0.5 + 0.1 * np.random.rand(len(t), 4)
    rep = analyze_run(t, vx, vy, r, delta, 2.7, utilization=util)
    ok = np.isfinite(rep.steady.understeer_gradient_deg_per_g)
    ok = ok and isinstance(rep.balance.classification, str)
    return ok, {"class": rep.balance.classification, "K": rep.steady.understeer_gradient_deg_per_g}


def run_phase72_validation() -> bool:
    print("=== Phase 7.2 Handling Metrics Validation ===\n")
    tests = [
        ("straight_line_metrics", test_straight_line_metrics),
        ("constant_radius_corner", test_constant_radius_corner),
        ("tire_utilization_bounds", test_utilization_bounds),
        ("understeer_classification", test_understeer_classification),
        ("oversteer_classification", test_oversteer_classification),
        ("neutral_classification", test_neutral_classification),
        ("k_formula", test_k_formula),
        ("regression_smoke", test_regression_smoke),
        ("no_nan_inf", test_no_nan),
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
    run_phase72_validation()
