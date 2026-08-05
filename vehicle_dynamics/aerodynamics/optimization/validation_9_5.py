"""Phase 9.5 – Aero optimization validation (target 13/13)."""

from __future__ import annotations

import numpy as np

from vehicle_dynamics.aerodynamics.coefficients import AeroConfig
from vehicle_dynamics.aerodynamics.aero_model import compute_aero_loads
from vehicle_dynamics.aerodynamics.ride_height import RideHeightState

from .design_variables import DesignVector, default_bounds
from .objective_functions import evaluate_objectives
from .constraints import evaluate_constraints, ConstraintSet
from .pareto import pareto_front, dominates
from .nsga2 import NSGA2Config, nsga2_optimize
from .surrogate_model import train_surrogate
from .lap_time_optimizer import optimize_lap_time, LapTimeModel
from .ai_explorer import AIExplorer


def test_baseline_regression() -> tuple[bool, dict]:
    """Default design objectives finite and match plant."""
    d = DesignVector()
    o = evaluate_objectives(d, speed=50.0, use_devices=False)
    st = compute_aero_loads(50.0, AeroConfig(), ride=RideHeightState())
    ok = abs(o.drag - st.drag) < 1e-6
    ok = ok and abs(o.downforce - st.downforce_total) < 1e-6
    return ok, {"drag": o.drag, "DF": o.downforce}


def test_pareto_generation() -> tuple[bool, dict]:
    rng = np.random.default_rng(0)
    objs = np.column_stack([
        rng.uniform(2000, 5000, 30),
        rng.uniform(400, 1200, 30),
    ])
    idx = pareto_front(objs, maximize_mask=np.array([True, False]))
    ok = len(idx) >= 1
    # No Pareto point dominated by another in set
    for i in idx:
        for j in idx:
            if i != j:
                if dominates(objs[j], objs[i], np.array([True, False])):
                    ok = False
    return ok, {"n_pareto": len(idx)}


def test_drag_downforce_tradeoff() -> tuple[bool, dict]:
    low_wing = DesignVector(rear_wing_angle=0.05, front_wing_angle=0.05)
    high_wing = DesignVector(rear_wing_angle=0.22, front_wing_angle=0.18)
    o1 = evaluate_objectives(low_wing, speed=50.0)
    o2 = evaluate_objectives(high_wing, speed=50.0)
    ok = o2.downforce > o1.downforce and o2.drag > o1.drag
    return ok, {"DF1": o1.downforce, "DF2": o2.downforce, "D1": o1.drag, "D2": o2.drag}


def test_lap_time_improvement() -> tuple[bool, dict]:
    baseline = LapTimeModel().evaluate(DesignVector())
    res = optimize_lap_time(n_samples=40, n_refine=15, seed=1)
    ok = res.best_time <= baseline + 1e-6
    ok = ok and res.best_time < baseline * 1.05  # not wildly worse
    return ok, {"baseline": baseline, "best": res.best_time}


def test_constraint_enforcement() -> tuple[bool, dict]:
    bad = DesignVector(h_front=0.01, h_rear=0.01, rear_wing_angle=0.5)
    o = evaluate_objectives(bad, speed=50.0)
    ok_feas, viol = evaluate_constraints(bad, o.drag, o.front_balance)
    ok = not ok_feas and viol["h_front"] > 0
    return ok, {"feasible": ok_feas, "h_front_viol": viol["h_front"]}


def test_surrogate_accuracy() -> tuple[bool, dict]:
    bounds = default_bounds()
    rng = np.random.default_rng(2)
    designs, objs = [], []
    for _ in range(25):
        d = bounds.random(rng)
        o = evaluate_objectives(d, speed=50.0)
        designs.append(d)
        objs.append([o.downforce, o.drag])
    surr = train_surrogate(designs, np.asarray(objs))
    # Predict training point ≈ exact
    pred = surr.predict_design(designs[0])
    err = np.linalg.norm(pred - objs[0]) / (np.linalg.norm(objs[0]) + 1e-9)
    ok = err < 0.05
    return ok, {"rel_err": err}


def test_ai_candidate_generation() -> tuple[bool, dict]:
    exp = AIExplorer(seed=3).explore(n_seed=15, n_candidates=5, evaluate_true=True)
    ok = len(exp.candidates) >= 1
    ok = ok and exp.true_objs is not None and np.all(np.isfinite(exp.true_objs))
    return ok, {"n": len(exp.candidates)}


def test_optimizer_repeatability() -> tuple[bool, dict]:
    r1 = nsga2_optimize(config=NSGA2Config(pop_size=16, n_gen=4, seed=7))
    r2 = nsga2_optimize(config=NSGA2Config(pop_size=16, n_gen=4, seed=7))
    ok = np.allclose(r1.objectives, r2.objectives)
    return ok, {"match": ok}


def test_hybrid_cfd_integration() -> tuple[bool, dict]:
    """Objectives still run when devices disabled (analytical/CFD hybrid path)."""
    d = DesignVector()
    o = evaluate_objectives(d, speed=40.0, use_devices=False)
    ok = o.downforce > 0 and o.drag > 0 and np.isfinite(o.lap_time)
    return ok, {"DF": o.downforce, "lap": o.lap_time}


def test_active_aero_optimization() -> tuple[bool, dict]:
    """DRS schedule in (0,1) can reduce lap-time proxy."""
    off = DesignVector(drs_schedule=0.0)
    on = DesignVector(drs_schedule=0.8)
    t0 = evaluate_objectives(off, speed=55.0).lap_time
    t1 = evaluate_objectives(on, speed=55.0).lap_time
    ok = t1 < t0
    return ok, {"t_drs_off": t0, "t_drs_on": t1}


def test_performance_scaling() -> tuple[bool, dict]:
    d = DesignVector()
    o1 = evaluate_objectives(d, speed=30.0)
    o2 = evaluate_objectives(d, speed=60.0)
    # Forces ~ V^2
    r_df = o2.downforce / max(o1.downforce, 1e-9)
    ok = 3.5 < r_df < 4.5
    return ok, {"DF_ratio": r_df}


def test_no_nan_inf() -> tuple[bool, dict]:
    res = nsga2_optimize(config=NSGA2Config(pop_size=12, n_gen=3, seed=0))
    ok = np.all(np.isfinite(res.objectives))
    for d in res.population:
        o = evaluate_objectives(d)
        if not all(np.isfinite([o.downforce, o.drag, o.lap_time])):
            ok = False
    return ok, {"ok": ok}


def test_regression_contract() -> tuple[bool, dict]:
    """optimization_enabled=False equivalent: plant unchanged (devices off path)."""
    d = DesignVector()
    o = evaluate_objectives(d, speed=50.0, use_devices=False)
    st = compute_aero_loads(
        50.0, AeroConfig(),
        ride=RideHeightState(h_front=d.h_front, h_rear=d.h_rear),
    )
    ok = abs(o.downforce - st.downforce_total) < 1e-6
    return ok, {"ok": ok}


def run_phase95_validation() -> bool:
    print("=== Phase 9.5 Aerodynamic Optimization Validation ===\n")
    tests = [
        ("baseline_regression", test_baseline_regression),
        ("pareto_generation", test_pareto_generation),
        ("drag_downforce_tradeoff", test_drag_downforce_tradeoff),
        ("lap_time_improvement", test_lap_time_improvement),
        ("constraint_enforcement", test_constraint_enforcement),
        ("surrogate_accuracy", test_surrogate_accuracy),
        ("ai_candidate_generation", test_ai_candidate_generation),
        ("optimizer_repeatability", test_optimizer_repeatability),
        ("hybrid_cfd_integration", test_hybrid_cfd_integration),
        ("active_aero_optimization", test_active_aero_optimization),
        ("performance_scaling", test_performance_scaling),
        ("no_nan_inf", test_no_nan_inf),
        ("regression_contract", test_regression_contract),
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
        print("Phase 9.5 Status: VALIDATED ✓")
    return all_pass


if __name__ == "__main__":
    run_phase95_validation()
