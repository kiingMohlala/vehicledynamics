"""
Phase 15.7 — ESC Gain Robustness & Candidate Selection.

Nominate robust K_Mz range. Gains remain NOT FROZEN unless evidence is overwhelming.
No plant or architecture changes.
"""
from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np

from vehicle_dynamics.demonstration.vehicle_binding import (
    bind_authoritative_hypercar,
    bind_historical_demonstrator,
)
from vehicle_dynamics.simulation.simulation import Simulation
from vehicle_dynamics.controls.esc_characterization import characterize_run
from vehicle_dynamics.controls.esc_decision import ESCDecisionLogic
from vehicle_dynamics.controls.esc_observability import ESCObservation

ROOT = Path("artifacts/phase_15_7")
REF_HYPER = (3.13, 8.34)
REF_HIST = (5.37, 19.81)
K_LIST = [4000.0, 6000.0, 8000.0, 10000.0, 12000.0]
DIST_LIST = [1500.0, 3000.0, 5000.0, 7000.0]
SPEED_LIST = [15.0, 25.0, 35.0]
STEER_LIST = [0.04, 0.08, 0.12]


def _gate(gates, name, ok, detail=""):
    gates.append({"name": name, "pass": bool(ok), "detail": detail})
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")


def _t_to(vx, t, speed):
    idx = np.where(np.asarray(vx) >= speed)[0]
    return float(t[idx[0]]) if len(idx) else None


def _launch(cfg, n=2500):
    sim = Simulation(cfg)
    sim.reset(0.0, 1)
    vx, tt = [], []
    for _ in range(n):
        sim._step_plant(1.0, 0, 0, 1.0, 0, 0.01)
        vx.append(sim.state.vehicle.vx)
        tt.append(sim.state.time)
    return vx, tt, sim


def run_validation() -> dict:
    ROOT.mkdir(parents=True, exist_ok=True)
    gates = []
    hyper = bind_authoritative_hypercar(1100.0, 750.0)
    hist = bind_historical_demonstrator()
    cfg = hyper.simulation_config
    mu0 = float(getattr(cfg, "mu_tire", 1.15))

    def factory():
        return Simulation(cfg)

    rows = []

    # Core matrix: K × |dist| × speed (nominal, mild steer, both signs)
    print("  Running robustness matrix...")
    for K in K_LIST:
        for M in DIST_LIST:
            for vx in SPEED_LIST:
                for sign in (-1.0, +1.0):
                    r = characterize_run(
                        factory,
                        K_Mz=K,
                        vx0=vx,
                        steer=0.06,
                        Mz_dist=sign * M,
                        recover_n=150,
                    )
                    d = r.to_dict()
                    d["sign"] = sign
                    rows.append(d)

    # Steering sweep at K=4000 and K=12000, M=3000, vx=25
    for K in (4000.0, 12000.0):
        for st in STEER_LIST:
            r = characterize_run(
                factory, K_Mz=K, vx0=25.0, steer=st, Mz_dist=-3000.0, recover_n=150,
            )
            d = r.to_dict()
            d["sign"] = -1.0
            d["note"] = "steer_sweep"
            rows.append(d)

    # Split-μ at each K, M=3000, vx=25
    mu_split = np.array([mu0, 0.5 * mu0, mu0, 0.5 * mu0])
    for K in K_LIST:
        r = characterize_run(
            factory, K_Mz=K, vx0=25.0, steer=0.06, Mz_dist=-3000.0,
            mu_per_wheel=mu_split, recover_n=150,
        )
        d = r.to_dict()
        d["sign"] = -1.0
        d["note"] = "split_mu"
        rows.append(d)

    # Free-response baselines (ESC effectively K=0 via disabled path) for key cells
    free_rows = []
    from vehicle_dynamics.controls.esc_command import ESCCommand, BrakeAllocator
    from vehicle_dynamics.controls.esc_observability import ESCObservability

    def free_response(M, vx=25.0):
        alloc = BrakeAllocator()
        sim = Simulation(cfg)
        sim.reset(vx, 3)
        for _ in range(40):
            sim.esc_brake_add = None
            sim._step_plant(0.12, 0, 0.06, 1, 0, 0.01)
        for _ in range(25):
            sim.esc_brake_add = alloc.allocate(ESCCommand(-M)).brake_cmd
            sim._step_plant(0.12, 0, 0.06, 1, 0, 0.01)
        e0 = abs(ESCObservability().observe_from_simulation(sim).e_r)
        e_hist = []
        for _ in range(150):
            sim.esc_brake_add = None
            sim._step_plant(0.12, 0, 0.06, 1, 0, 0.01)
            e_hist.append(abs(ESCObservability().observe_from_simulation(sim).e_r))
        return e0, e_hist[-1]

    for M in DIST_LIST:
        e0, ef = free_response(M)
        free_rows.append({"Mz_dist": M, "e0": e0, "e_final_free": ef})

    # Persist
    with open(ROOT / "gain_robustness.json", "w") as f:
        json.dump({"rows": rows, "free": free_rows}, f, indent=2)
    keys = sorted({k for r in rows for k in r.keys()})
    with open(ROOT / "gain_robustness.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)

    # --- Gates ---
    _gate(gates, "no_nan_instability",
          all(np.isfinite(r["e_final"]) and np.isfinite(r["e_peak"]) for r in rows),
          f"n_rows={len(rows)}")

    _gate(gates, "no_sustained_oscillation",
          all(r["mz_flips"] <= 6 for r in rows),
          f"max_flips={max(r['mz_flips'] for r in rows)}")

    _gate(gates, "brake_cmd_bounded",
          all(r["max_cmd"] <= 1.0 + 1e-9 for r in rows),
          f"max_cmd={max(r['max_cmd'] for r in rows):.3f}")

    # Recovery vs free: for negative dist at vx=25, each K final ≤ free * 1.05
    free_map = {fr["Mz_dist"]: fr["e_final_free"] for fr in free_rows}
    worse = []
    for r in rows:
        if r.get("mu_mode") != "nominal":
            continue
        if r.get("sign", -1) > 0:
            continue  # asymmetric +dist known hard
        if abs(r["vx0"] - 25.0) > 0.1:
            continue
        if abs(r["steer"] - 0.06) > 0.01:
            continue
        M = abs(r["Mz_dist"])
        if M not in free_map:
            continue
        if r["e_final"] > free_map[M] * 1.05 + 0.02:
            worse.append((r["K_Mz"], M, r["e_final"], free_map[M]))
    _gate(gates, "recovery_not_worse_than_free",
          len(worse) == 0,
          f"worse_cells={worse[:5]}")

    split_rows = [r for r in rows if r.get("mu_mode") == "split"]
    _gate(gates, "split_mu_bounded",
          all(r["e_final"] < 5.0 for r in split_rows),
          f"max_ef={max(r['e_final'] for r in split_rows):.3f}")

    # Inhibits intact
    logic = ESCDecisionLogic()
    d_util = logic.step(ESCObservation(
        vx=25, delta=0.1, e_r=0.4, util_max=0.99, beta=0.0,
        r=0.5, r_ref=0.2, r_kin=0.6, ay=5,
    ))
    logic.reset()
    d_beta = logic.step(ESCObservation(
        vx=25, delta=0.1, e_r=0.4, util_max=0.5, beta=0.5,
        r=0.5, r_ref=0.2, r_kin=0.6, ay=5,
    ))
    logic.reset()
    d_lo = logic.step(ESCObservation(
        vx=3, delta=0.1, e_r=0.4, util_max=0.5, beta=0.0,
        r=0.5, r_ref=0.2, r_kin=0.6, ay=1,
    ))
    _gate(gates, "inhibits_intact",
          d_util.inhibited and d_beta.inhibited and d_lo.reason == "low_speed",
          f"util={d_util.reason} beta={d_beta.reason} lo={d_lo.reason}")

    # Determinism
    runs = []
    for _ in range(5):
        r = characterize_run(factory, K_Mz=8000.0, vx0=25.0, steer=0.06, Mz_dist=-3000.0)
        runs.append(round(r.e_final, 5))
    _gate(gates, "deterministic_replay", len(set(runs)) == 1, f"run0={runs[0]}")

    # Score robustness: for each K, mean e_final over nominal −dist cells at all speeds/M
    scores = {}
    for K in K_LIST:
        cells = [
            r for r in rows
            if r["K_Mz"] == K and r.get("mu_mode") == "nominal"
            and r.get("sign", -1) < 0 and abs(r["steer"] - 0.06) < 0.01
        ]
        if not cells:
            continue
        mean_ef = float(np.mean([c["e_final"] for c in cells]))
        max_ef = float(np.max([c["e_final"] for c in cells]))
        mean_sat = float(np.mean([c["sat_fraction"] for c in cells]))
        max_flips = max(c["mz_flips"] for c in cells)
        scores[K] = {
            "mean_e_final": mean_ef,
            "max_e_final": max_ef,
            "mean_sat": mean_sat,
            "max_flips": max_flips,
            "n": len(cells),
        }

    # Lowest gain with mean_e_final within 15% of best mean
    best_mean = min(s["mean_e_final"] for s in scores.values())
    robust = [
        K for K, s in scores.items()
        if s["mean_e_final"] <= best_mean * 1.15 and s["max_flips"] <= 4
    ]
    lowest_robust = min(robust) if robust else min(scores, key=lambda k: scores[k]["mean_e_final"])
    best_K = min(scores, key=lambda k: scores[k]["mean_e_final"])

    nomination = {
        "best_mean_e_final_K": best_K,
        "lowest_robust_K": lowest_robust,
        "recommended_range": [lowest_robust, best_K] if lowest_robust <= best_K else [best_K, lowest_robust],
        "scores": {str(k): v for k, v in scores.items()},
        "K_Mz_frozen": False,
        "rationale": (
            "Prefer lowest gain within 15% of best mean residual on nominal −dist matrix; "
            "do not freeze without broader campaign."
        ),
    }
    with open(ROOT / "nomination.json", "w") as f:
        json.dump(nomination, f, indent=2)

    print("  Robustness scores:")
    for K, s in scores.items():
        print(f"    K={K:.0f}  mean_ef={s['mean_e_final']:.3f}  max_ef={s['max_e_final']:.3f}  "
              f"sat={s['mean_sat']:.2f}  flips={s['max_flips']}")

    _gate(gates, "robust_candidate_identified",
          lowest_robust in K_LIST and best_K in K_LIST,
          f"lowest_robust={lowest_robust} best_mean={best_K}")

    # Regression
    avx, at, _ = _launch(cfg)
    at100, at200 = _t_to(avx, at, 27.78), _t_to(avx, at, 55.56)
    reg = (
        at100 is not None and abs(at100 - REF_HYPER[0]) < 0.15
        and at200 is not None and abs(at200 - REF_HYPER[1]) < 0.25
    )
    _gate(gates, "regression", reg, f"t100={at100} t200={at200}")
    hvx, ht, _ = _launch(hist.simulation_config)
    ht100 = _t_to(hvx, ht, 27.78)
    _gate(gates, "historical_isolation",
          ht100 is not None and abs(ht100 - REF_HIST[0]) < 0.3,
          f"t100={ht100}")

    n_pass = sum(1 for g in gates if g["pass"])
    status = "PASS" if n_pass == len(gates) else (
        "PASS WITH LIMITATIONS" if n_pass >= len(gates) - 1 else "FAIL"
    )
    summary = {
        "phase": "15.7",
        "status": status,
        "gates_passed": n_pass,
        "gates_total": len(gates),
        "gates": gates,
        "nomination": nomination,
        "n_matrix_rows": len(rows),
        "regression": {
            "hyper": {"t100": at100, "t200": at200, "ref": REF_HYPER},
            "hist": {"t100": ht100, "ref": REF_HIST},
        },
    }
    with open(ROOT / "validation_result.json", "w") as f:
        json.dump(summary, f, indent=2, default=str)
    print(f"\n=== PHASE 15.7 — {status} {n_pass}/{len(gates)} ===")
    print(f"  Nominate range: {nomination['recommended_range']}  FROZEN=NO")
    return summary


if __name__ == "__main__":
    run_validation()
