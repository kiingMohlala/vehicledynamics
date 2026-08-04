"""
Phase 7.4 – Relaxation length validation.
"""

from __future__ import annotations

import numpy as np
from .dugoff import DugoffTire, DugoffParams
from .pacejka import PacejkaTire
from .relaxation import step_relaxation
from .relaxation_parameters import RelaxationParams, disabled
from .relaxation_state import RelaxationState
from .transient_tire import TransientTire


def test_disabled_equals_baseline() -> tuple[bool, dict]:
    steady = DugoffTire()
    wrap = TransientTire(steady, disabled())
    errs = []
    for k in (0.0, 0.05, 0.2, -0.1):
        for a in (0.0, 0.05, -0.08):
            s0 = steady.longitudinal_lateral_force(k, a, 4000.0)
            s1 = wrap.update(k, a, 4000.0, vx=20.0, dt=0.01)
            errs.append(abs(s0.Fx - s1.Fx) + abs(s0.Fy - s1.Fy))
    ok = max(errs) < 1e-9
    return ok, {"max_err": float(max(errs))}


def test_zero_length_baseline() -> tuple[bool, dict]:
    steady = DugoffTire()
    wrap = TransientTire(steady, RelaxationParams(enabled=True, Lx=1e-6, Ly=1e-6))
    wrap.reset(0.0, 0.0)
    s0 = steady.longitudinal_lateral_force(0.2, 0.1, 4000.0)
    s1 = wrap.update(0.2, 0.1, 4000.0, vx=20.0, dt=0.01)
    err = abs(s0.Fx - s1.Fx) + abs(s0.Fy - s1.Fy)
    ok = err < 1.0
    return ok, {"err": err}


def test_steering_step_lag() -> tuple[bool, dict]:
    """Alpha_eff lags; force converges to steady-state."""
    params = RelaxationParams(Lx=0.3, Ly=0.8)
    st = RelaxationState(0.0, 0.0)
    alphas = []
    fy = []
    steady = DugoffTire()
    for _ in range(80):
        st = step_relaxation(st, 0.0, 0.1, 12.0, 0.01, params)
        alphas.append(st.alpha_eff)
        fy.append(steady.longitudinal_lateral_force(0.0, st.alpha_eff, 4000.0).Fy)
    s_ss = steady.longitudinal_lateral_force(0.0, 0.1, 4000.0)
    # Effective slip clearly lags on first steps
    ok = alphas[0] < 0.05 and alphas[5] < 0.09
    ok = ok and alphas[-1] > 0.095
    ok = ok and abs(fy[-1] - s_ss.Fy) / (abs(s_ss.Fy) + 1e-6) < 0.05
    ok = ok and fy[0] < fy[-1]  # force builds up
    return ok, {
        "alpha_0": alphas[0],
        "alpha_5": alphas[5],
        "alpha_last": alphas[-1],
        "Fy_first": fy[0],
        "Fy_last": fy[-1],
        "Fy_ss": s_ss.Fy,
    }


def test_braking_step_lag() -> tuple[bool, dict]:
    params = RelaxationParams(Lx=0.8, Ly=0.5)
    st = RelaxationState(0.0, 0.0)
    kappas = []
    fx = []
    steady = DugoffTire()
    for _ in range(80):
        st = step_relaxation(st, 0.15, 0.0, 12.0, 0.01, params)
        kappas.append(st.kappa_eff)
        fx.append(steady.longitudinal_lateral_force(st.kappa_eff, 0.0, 4000.0).Fx)
    s_ss = steady.longitudinal_lateral_force(0.15, 0.0, 4000.0)
    ok = kappas[0] < 0.08 and kappas[5] < 0.14
    ok = ok and kappas[-1] > 0.14
    ok = ok and abs(fx[-1] - s_ss.Fx) / (abs(s_ss.Fx) + 1e-6) < 0.05
    ok = ok and abs(fx[0]) < abs(fx[-1])
    return ok, {
        "kappa_0": kappas[0],
        "kappa_5": kappas[5],
        "kappa_last": kappas[-1],
        "Fx_first": fx[0],
        "Fx_last": fx[-1],
        "Fx_ss": s_ss.Fx,
    }


def test_steady_state_convergence() -> tuple[bool, dict]:
    steady = PacejkaTire()
    wrap = TransientTire(steady, RelaxationParams(Lx=0.3, Ly=0.5))
    wrap.reset(0.0, 0.0)
    for _ in range(200):
        wrap.update(0.08, -0.06, 3500.0, vx=20.0, dt=0.01)
    s_ss = steady.longitudinal_lateral_force(0.08, -0.06, 3500.0)
    s_tr = wrap.update(0.08, -0.06, 3500.0, vx=20.0, dt=0.01)
    err = abs(s_ss.Fx - s_tr.Fx) + abs(s_ss.Fy - s_tr.Fy)
    ok = err < 1.0
    return ok, {"err": err, "Fx": s_tr.Fx, "Fy": s_tr.Fy}


def test_higher_speed_faster() -> tuple[bool, dict]:
    params = RelaxationParams(Lx=0.5, Ly=0.5)

    def settle_time(vx: float) -> int:
        st = RelaxationState(0.0, 0.0)
        target = 0.2
        for i in range(500):
            st = step_relaxation(st, target, 0.0, vx, 0.01, params)
            if abs(st.kappa_eff - target) < 0.01 * abs(target):
                return i
        return 500

    t_slow = settle_time(5.0)
    t_fast = settle_time(30.0)
    ok = t_fast < t_slow
    return ok, {"steps_5mps": t_slow, "steps_30mps": t_fast}


def test_symmetry() -> tuple[bool, dict]:
    steady = DugoffTire()
    w1 = TransientTire(steady, RelaxationParams(Lx=0.3, Ly=0.5))
    w2 = TransientTire(steady, RelaxationParams(Lx=0.3, Ly=0.5))
    w1.reset()
    w2.reset()
    for _ in range(30):
        s1 = w1.update(0.1, 0.08, 4000.0, vx=18.0, dt=0.01)
        s2 = w2.update(-0.1, -0.08, 4000.0, vx=18.0, dt=0.01)
    ok = abs(s1.Fx + s2.Fx) < 1e-6 and abs(s1.Fy + s2.Fy) < 1e-6
    return ok, {"Fx_sum": s1.Fx + s2.Fx, "Fy_sum": s1.Fy + s2.Fy}


def test_numerical_robustness() -> tuple[bool, dict]:
    steady = DugoffTire()
    wrap = TransientTire(steady, RelaxationParams())
    ok = True
    for vx in (0.0, 0.1, 5.0, 40.0):
        wrap.reset()
        for _ in range(10):
            s = wrap.update(0.5, -0.4, 100.0, vx=vx, dt=0.02)
            if not all(np.isfinite([s.Fx, s.Fy, s.utilization])):
                ok = False
    return ok, {}


def test_no_nan_inf() -> tuple[bool, dict]:
    return test_numerical_robustness()


def test_api_steady_path() -> tuple[bool, dict]:
    steady = DugoffTire()
    wrap = TransientTire(steady, RelaxationParams(Lx=0.5, Ly=0.5))
    wrap.reset(0.0, 0.0)
    wrap.update(0.2, 0.1, 4000.0, vx=10.0, dt=0.01)
    s_api = wrap.longitudinal_lateral_force(0.2, 0.1, 4000.0)
    s_ss = steady.longitudinal_lateral_force(0.2, 0.1, 4000.0)
    ok = abs(s_api.Fx - s_ss.Fx) < 1e-9 and abs(s_api.Fy - s_ss.Fy) < 1e-9
    return ok, {"Fx": s_api.Fx}


def run_phase74_validation() -> bool:
    print("=== Phase 7.4 Tire Relaxation Length Validation ===\n")
    tests = [
        ("disabled_equals_baseline", test_disabled_equals_baseline),
        ("zero_length_baseline", test_zero_length_baseline),
        ("steering_step_lag", test_steering_step_lag),
        ("braking_step_lag", test_braking_step_lag),
        ("steady_state_convergence", test_steady_state_convergence),
        ("higher_speed_faster", test_higher_speed_faster),
        ("left_right_symmetry", test_symmetry),
        ("numerical_robustness", test_numerical_robustness),
        ("no_nan_inf", test_no_nan_inf),
        ("api_steady_path", test_api_steady_path),
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
    run_phase74_validation()
