"""
Phase 4.0 – Independent validation for the dynamic bicycle model.

Includes smoke tests plus physics checks required before freeze:
- Steady-state circular
- Left/right symmetry
- Linear bicycle cross-check
- Dual lateral-acceleration consistency
"""

import numpy as np
from .parameters import BicycleParameters
from .bicycle import DynamicBicycleModel
from ..tire.dugoff import DugoffParams

def test_straight_line_stability():
    model = DynamicBicycleModel()
    res = model.simulate(vx=20.0, t_span=(0, 5), delta_func=lambda t: 0.0)
    ok = np.max(np.abs(res.vy)) < 1e-3 and np.max(np.abs(res.r)) < 1e-3
    return ok, {"max_vy": float(np.max(np.abs(res.vy))), "max_r": float(np.max(np.abs(res.r)))}

def test_step_steer_response():
    model = DynamicBicycleModel()
    delta_final = np.deg2rad(3.0)
    def delta_func(t):
        return delta_final if t >= 1.0 else 0.0
    res = model.simulate(vx=20.0, t_span=(0, 8), delta_func=delta_func)
    post = res.time >= 2.0
    ok = (
        np.all(np.isfinite(res.vy)) and np.all(np.isfinite(res.r)) and
        np.max(np.abs(res.r[post])) > 1e-3 and np.max(np.abs(res.vy[post])) > 1e-3
    )
    return ok, {
        "max_r_post": float(np.max(np.abs(res.r[post]))),
        "final_r": float(res.r[-1]),
        "final_vy": float(res.vy[-1]),
    }

def test_dual_ay_consistency():
    """ay_force and ay_vehicle must agree closely after transients."""
    model = DynamicBicycleModel()
    delta_final = np.deg2rad(2.0)
    res = model.simulate(
        vx=15.0, t_span=(0, 8),
        delta_func=lambda t: delta_final if t >= 0.5 else 0.0,
    )
    # Compare in the quasi-steady region
    mask = res.time > 3.0
    err = np.max(np.abs(res.ay_force[mask] - res.ay_vehicle[mask]))
    ok = err < 0.15   # allow small numerical gradient error
    return ok, {"max_ay_error": float(err)}

def test_left_right_symmetry():
    model = DynamicBicycleModel()
    delta = np.deg2rad(3.0)

    res_pos = model.simulate(
        vx=18.0, t_span=(0, 8),
        delta_func=lambda t: delta if t >= 0.5 else 0.0,
    )
    res_neg = model.simulate(
        vx=18.0, t_span=(0, 8),
        delta_func=lambda t: -delta if t >= 0.5 else 0.0,
    )

    # Compare final steady values
    ok = (
        abs(res_pos.r[-1] + res_neg.r[-1]) < 1e-3 and
        abs(res_pos.vy[-1] + res_neg.vy[-1]) < 1e-3 and
        abs(res_pos.Fy_f[-1] + res_neg.Fy_f[-1]) < 5.0 and
        abs(res_pos.Fy_r[-1] + res_neg.Fy_r[-1]) < 5.0
    )
    return ok, {
        "r_sum": float(res_pos.r[-1] + res_neg.r[-1]),
        "vy_sum": float(res_pos.vy[-1] + res_neg.vy[-1]),
    }

def test_steady_circular():
    """Constant steer → yaw rate and lateral accel settle to near-constant values."""
    model = DynamicBicycleModel()
    delta = np.deg2rad(2.5)
    res = model.simulate(
        vx=15.0, t_span=(0, 12),
        delta_func=lambda t: delta,
    )
    # Last 2 seconds should be nearly steady
    mask = res.time > 10.0
    r_std = float(np.std(res.r[mask]))
    ay_std = float(np.std(res.ay_force[mask]))
    ok = r_std < 0.01 and ay_std < 0.15 and abs(res.r[-1]) > 1e-3
    # Approximate radius
    R = 15.0 / res.r[-1] if abs(res.r[-1]) > 1e-6 else np.inf
    return ok, {
        "final_r": float(res.r[-1]),
        "final_ay": float(res.ay_force[-1]),
        "r_std": r_std,
        "approx_radius_m": float(R),
    }

def test_linear_regime_crosscheck():
    """
    Small steer angles: compare steady yaw-rate gain against
    classical linear bicycle analytic result.

    Linear bicycle steady-state yaw gain:
        r / δ = Vx / (L + K_us * Vx²)
    where K_us = (m/L) * (b/Cf - a/Cr)
    Using Cx/Cy from the tire model as effective cornering stiffnesses.
    """
    # Use relatively stiff tires so the linear regime is wider
    tire_params = DugoffParams(mu=1.2, Cx=90000.0, Cy=90000.0)
    model = DynamicBicycleModel(tire_params=tire_params)
    p = model.p
    Cf = tire_params.Cy   # N/rad
    Cr = tire_params.Cy

    Kus = (p.m / p.L) * (p.b / Cf - p.a / Cr)
    vx = 12.0
    analytic_gain = vx / (p.L + Kus * vx**2)

    errors = []
    for deg in [0.5, 1.0, 1.5]:
        delta = np.deg2rad(deg)
        res = model.simulate(
            vx=vx, t_span=(0, 10),
            delta_func=lambda t, d=delta: d,
        )
        r_ss = res.r[-1]
        sim_gain = r_ss / delta
        errors.append(abs(sim_gain - analytic_gain) / abs(analytic_gain))

    rms_err = float(np.sqrt(np.mean(np.array(errors)**2)))
    max_err = float(np.max(errors))
    # Allow generous tolerance because Dugoff is not purely linear
    ok = max_err < 0.35
    return ok, {
        "analytic_gain": float(analytic_gain),
        "rms_relative_error": rms_err,
        "max_relative_error": max_err,
    }

def test_no_nan_inf():
    model = DynamicBicycleModel()
    res = model.simulate(
        vx=25.0, t_span=(0, 5),
        delta_func=lambda t: np.deg2rad(4.0) if t > 0.2 else 0.0,
    )
    arrays = [res.vy, res.r, res.alpha_f, res.alpha_r, res.Fy_f, res.Fy_r,
              res.ay_force, res.ay_vehicle]
    ok = all(np.all(np.isfinite(a)) for a in arrays)
    return ok, {}

def run_all_tests():
    print("=== Phase 4.0 Bicycle Model Validation ===\n")
    tests = [
        ("Straight-line stability", test_straight_line_stability),
        ("Step-steer response", test_step_steer_response),
        ("Dual ay consistency", test_dual_ay_consistency),
        ("Left/right symmetry", test_left_right_symmetry),
        ("Steady-state circular", test_steady_circular),
        ("Linear regime cross-check", test_linear_regime_crosscheck),
        ("No NaN/Inf", test_no_nan_inf),
    ]

    results = {}
    all_pass = True
    for name, fn in tests:
        ok, diag = fn()
        results[name] = ok
        status = "PASS" if ok else "FAIL"
        print(f"{name:30} : {status}")
        if diag:
            for k, v in diag.items():
                print(f"    {k}: {v}")
        if not ok:
            all_pass = False

    print("\nOverall:", "ALL PASSED" if all_pass else "SOME FAILED")
    return all_pass, results

if __name__ == "__main__":
    run_all_tests()
