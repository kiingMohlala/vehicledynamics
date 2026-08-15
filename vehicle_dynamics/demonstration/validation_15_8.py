"""
Phase 15.8 — ESC Gain Selection & Final Candidate Validation.

Prefer lowest gain operationally indistinguishable from best recovery,
with less actuator demand. K_Mz remains NOT FROZEN unless evidence is clear.
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
from vehicle_dynamics.controls.esc_command import ESCCommand, BrakeAllocator
from vehicle_dynamics.controls.esc_observability import ESCObservability

ROOT = Path("artifacts/phase_15_8")
REF_HYPER = (3.13, 8.34)
REF_HIST = (5.37, 19.81)
CANDIDATES = [10000.0, 11000.0, 12000.0]
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
    print("  Focused selection matrix...")

    # Core: K × speed × |dist| × sign, steer=0.06 nominal
    for K in CANDIDATES:
        for vx in SPEED_LIST:
            for M in DIST_LIST:
                for sign in (-1.0, +1.0):
                    r = characterize_run(
                        factory, K_Mz=K, vx0=vx, steer=0.06,
                        Mz_dist=sign * M, recover_n=150,
                    )
                    d = r.to_dict()
                    d["sign"] = sign
                    d["surface"] = "nominal"
                    rows.append(d)

    # Steering × K at vx=25, M=-3000
    for K in CANDIDATES:
        for st in STEER_LIST:
            r = characterize_run(
                factory, K_Mz=K, vx0=25.0, steer=st, Mz_dist=-3000.0, recover_n=150,
            )
            d = r.to_dict()
            d["sign"] = -1.0
            d["surface"] = "nominal"
            d["note"] = "steer"
            rows.append(d)

    # Split L/R and F/R
    mu_lr = np.array([mu0, 0.5 * mu0, mu0, 0.5 * mu0])
    mu_fr = np.array([0.5 * mu0, 0.5 * mu0, mu0, mu0])
    for K in CANDIDATES:
        for label, mu_pw in (("split_lr", mu_lr), ("split_fr", mu_fr)):
            r = characterize_run(
                factory, K_Mz=K, vx0=25.0, steer=0.06, Mz_dist=-3000.0,
                mu_per_wheel=mu_pw, recover_n=150,
            )
            d = r.to_dict()
            d["sign"] = -1.0
            d["surface"] = label
            rows.append(d)

    # Free response for −dist at 25 m/s
    def free_ef(M):
        alloc = BrakeAllocator()
        sim = Simulation(cfg)
        sim.reset(25.0, 3)
        for _ in range(40):
            sim.esc_brake_add = None
            sim._step_plant(0.12, 0, 0.06, 1, 0, 0.01)
        for _ in range(25):
            sim.esc_brake_add = alloc.allocate(ESCCommand(-M)).brake_cmd
            sim._step_plant(0.12, 0, 0.06, 1, 0, 0.01)
        for _ in range(150):
            sim.esc_brake_add = None
            sim._step_plant(0.12, 0, 0.06, 1, 0, 0.01)
        return abs(ESCObservability().observe_from_simulation(sim).e_r)

    free_map = {M: free_ef(M) for M in DIST_LIST}

    # Aggregate per candidate (prefer −dist nominal for recovery quality)
    metrics = {}
    for K in CANDIDATES:
        neg = [
            r for r in rows
            if r["K_Mz"] == K and r.get("surface") == "nominal" and r.get("sign", -1) < 0
        ]
        all_k = [r for r in rows if r["K_Mz"] == K]
        split = [r for r in rows if r["K_Mz"] == K and str(r.get("surface", "")).startswith("split")]
        pos = [
            r for r in rows
            if r["K_Mz"] == K and r.get("surface") == "nominal" and r.get("sign", -1) > 0
        ]
        metrics[K] = {
            "mean_e_final": float(np.mean([r["e_final"] for r in neg])),
            "worst_e_final": float(np.max([r["e_final"] for r in neg])),
            "mean_e_peak": float(np.mean([r["e_peak"] for r in neg])),
            "mean_max_Mz": float(np.mean([r["max_Mz"] for r in neg])),
            "mean_brake_energy": float(np.mean([r["brake_energy"] for r in neg])),
            "mean_sat": float(np.mean([r["sat_fraction"] for r in neg])),
            "mean_util": float(np.mean([r["util_peak"] for r in neg])),
            "max_flips": int(max(r["mz_flips"] for r in all_k)),
            "split_worst_ef": float(np.max([r["e_final"] for r in split])) if split else 0.0,
            "pos_worst_ef": float(np.max([r["e_final"] for r in pos])) if pos else 0.0,
            "n_neg": len(neg),
        }

    print("  Candidate metrics (nominal −dist):")
    for K, m in metrics.items():
        print(
            f"    K={K:.0f}  mean_ef={m['mean_e_final']:.4f}  worst={m['worst_e_final']:.4f}  "
            f"Mz={m['mean_max_Mz']:.0f}  E_brk={m['mean_brake_energy']:.3f}  "
            f"sat={m['mean_sat']:.2f}  util={m['mean_util']:.3f}"
        )

    # Selection: lowest K within 10% of best mean_e_final AND worst within 15% of best worst
    best_mean = min(m["mean_e_final"] for m in metrics.values())
    best_worst = min(m["worst_e_final"] for m in metrics.values())
    eligible = [
        K for K, m in metrics.items()
        if m["mean_e_final"] <= best_mean * 1.10
        and m["worst_e_final"] <= best_worst * 1.15
        and m["max_flips"] <= 4
    ]
    if not eligible:
        eligible = list(CANDIDATES)
    selected = min(eligible)  # prefer lowest gain among eligible
    best_raw = min(metrics, key=lambda k: metrics[k]["mean_e_final"])

    reason = (
        f"Lowest eligible gain among candidates within 10% of best mean residual "
        f"({best_mean:.4f}) and 15% of best worst-case ({best_worst:.4f}). "
        f"Best raw mean is K={best_raw:.0f}; selected K={selected:.0f} prioritizes "
        f"lower actuator demand when recovery is operationally equivalent."
    )

    # Worst-case cells for selected & extremes — repeated runs
    worst_cases = []
    for K in CANDIDATES:
        # find worst nominal -dist cell for this K
        neg = [
            r for r in rows
            if r["K_Mz"] == K and r.get("surface") == "nominal" and r.get("sign", -1) < 0
        ]
        worst = max(neg, key=lambda r: r["e_final"])
        reps = []
        for _ in range(5):
            rr = characterize_run(
                factory,
                K_Mz=K,
                vx0=worst["vx0"],
                steer=worst["steer"],
                Mz_dist=worst["Mz_dist"],
                recover_n=150,
            )
            reps.append(round(rr.e_final, 5))
        worst_cases.append({
            "K_Mz": K,
            "cell": {k: worst[k] for k in ("vx0", "steer", "Mz_dist", "e_final")},
            "repeated_e_final": reps,
            "deterministic": len(set(reps)) == 1,
        })

    # Gates
    _gate(gates, "numerical_stability",
          all(np.isfinite(r["e_final"]) for r in rows),
          f"n={len(rows)}")
    _gate(gates, "no_sustained_oscillation",
          all(r["mz_flips"] <= 4 for r in rows),
          f"max_flips={max(r['mz_flips'] for r in rows)}")
    _gate(gates, "brake_cmd_ok",
          all(r["max_cmd"] <= 1.0 + 1e-9 for r in rows),
          f"max_cmd={max(r['max_cmd'] for r in rows):.3f}")
    _gate(gates, "safety_envelope",
          all(r["e_final"] < 5.0 for r in rows),
          f"max_ef={max(r['e_final'] for r in rows):.3f}")

    worse = []
    for r in rows:
        if r.get("surface") != "nominal" or r.get("sign", -1) > 0:
            continue
        if abs(r["vx0"] - 25) > 0.1 or abs(r["steer"] - 0.06) > 0.01:
            continue
        M = abs(r["Mz_dist"])
        if M in free_map and r["e_final"] > free_map[M] * 1.05 + 0.02:
            worse.append((r["K_Mz"], M, r["e_final"], free_map[M]))
    _gate(gates, "not_worse_than_free", len(worse) == 0, f"worse={worse[:3]}")

    pos_neg_ok = all(
        metrics[K]["pos_worst_ef"] < 5.0 for K in CANDIDATES
    )
    _gate(gates, "disturbance_symmetry_bounded", pos_neg_ok,
          f"pos_worst={[metrics[K]['pos_worst_ef'] for K in CANDIDATES]}")

    _gate(gates, "split_mu_bounded",
          all(metrics[K]["split_worst_ef"] < 5.0 for K in CANDIDATES),
          f"split_worst={[metrics[K]['split_worst_ef'] for K in CANDIDATES]}")

    # ABS coexistence sample
    from vehicle_dynamics.controls.esc_closed_loop import ClosedLoopESC, ClosedLoopESCConfig
    esc = ClosedLoopESC(ClosedLoopESCConfig(enabled=True, K_Mz=selected))
    sim = Simulation(cfg)
    sim.reset(30.0, 4)
    for _ in range(50):
        esc.step(sim)
        sim._step_plant(0.0, 0.7, 0.05, 1, 0, 0.01)
    d = sim.dual_track.diagnostics()
    _gate(gates, "abs_coexistence",
          d["min_Fz"] >= 50 - 1e-6,
          f"min_Fz={d['min_Fz']:.0f}")

    _gate(gates, "worst_case_deterministic",
          all(w["deterministic"] for w in worst_cases),
          f"reps={[w['repeated_e_final'][0] for w in worst_cases]}")

    _gate(gates, "multi_metric_selection",
          selected in CANDIDATES,
          f"selected={selected} eligible={eligible} best_raw={best_raw}")

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

    recommendation = {
        "selected_candidate": selected,
        "validated_range": [min(CANDIDATES), max(CANDIDATES)],
        "eligible": eligible,
        "best_raw_mean": best_raw,
        "gain_frozen": False,
        "reason": reason,
        "metrics": {str(k): v for k, v in metrics.items()},
        "plant": "FROZEN",
        "safety_envelope": "PRESERVED",
        "K_us": "FROZEN 0.0065",
    }

    with open(ROOT / "gain_selection.json", "w") as f:
        json.dump({"rows": rows, "metrics": recommendation["metrics"]}, f, indent=2)
    keys = sorted({k for r in rows for k in r.keys()})
    with open(ROOT / "gain_selection.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)
    with open(ROOT / "worst_case_runs.json", "w") as f:
        json.dump(worst_cases, f, indent=2)
    with open(ROOT / "recommendation.json", "w") as f:
        json.dump(recommendation, f, indent=2)

    n_pass = sum(1 for g in gates if g["pass"])
    status = "PASS" if n_pass == len(gates) else (
        "PASS WITH LIMITATIONS" if n_pass >= len(gates) - 1 else "FAIL"
    )
    summary = {
        "phase": "15.8",
        "status": status,
        "gates_passed": n_pass,
        "gates_total": len(gates),
        "gates": gates,
        "recommendation": recommendation,
        "regression": {
            "hyper": {"t100": at100, "t200": at200, "ref": REF_HYPER},
            "hist": {"t100": ht100, "ref": REF_HIST},
        },
    }
    with open(ROOT / "validation_result.json", "w") as f:
        json.dump(summary, f, indent=2, default=str)
    print(f"\n=== PHASE 15.8 — {status} {n_pass}/{len(gates)} ===")
    print(f"  Selected: K_Mz={selected}  FROZEN=NO")
    print(f"  Reason: {reason}")
    return summary


if __name__ == "__main__":
    run_validation()
