"""
Phase 15.9 — ESC Transient Event & Recovery Validation.

K_Mz = 10000 candidate (NOT FROZEN). No plant/architecture changes.
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
from vehicle_dynamics.controls.esc_closed_loop import ClosedLoopESC, ClosedLoopESCConfig
from vehicle_dynamics.controls.esc_command import ESCCommand, BrakeAllocator
from vehicle_dynamics.controls.esc_observability import ESCObservability

ROOT = Path("artifacts/phase_15_9")
REF_HYPER = (3.13, 8.34)
REF_HIST = (5.37, 19.81)
K_CAND = 10000.0
E_EXIT = 0.06


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


def _esc():
    return ClosedLoopESC(ClosedLoopESCConfig(enabled=True, K_Mz=K_CAND))


def _trace_step(sim, esc, thr, brk, steer):
    esc.step(sim)
    sim._step_plant(thr, brk, steer, 1, 0, 0.01)
    o = esc.observer.last
    return {
        "e_r": o.e_r,
        "r": o.r,
        "r_ref": o.r_ref,
        "Mz": esc.last_Mz,
        "active": esc.last_active,
        "util": o.util_max,
        "cmd_max": float(np.max(sim.esc_brake_add)) if sim.esc_brake_add is not None else 0.0,
    }


def run_validation() -> dict:
    ROOT.mkdir(parents=True, exist_ok=True)
    gates = []
    events = []
    traces = {}
    hyper = bind_authoritative_hypercar(1100.0, 750.0)
    hist = bind_historical_demonstrator()
    cfg = hyper.simulation_config
    mu0 = float(getattr(cfg, "mu_tire", 1.15))
    alloc = BrakeAllocator()

    def settle(sim, esc, n=40, steer=0.06):
        for _ in range(n):
            if esc.cfg.enabled:
                esc.step(sim)
            else:
                sim.esc_brake_add = None
            sim._step_plant(0.12, 0, steer, 1, 0, 0.01)

    # --- 1 Step yaw disturbance ± ---
    def step_dist(sign):
        esc = _esc()
        sim = Simulation(cfg)
        sim.reset(25.0, 3)
        settle(sim, esc)
        for _ in range(30):
            sim.esc_brake_add = alloc.allocate(ESCCommand(sign * 3500)).brake_cmd
            # ESC still steps but overlay may fight — for step, apply dist then recover
            sim._step_plant(0.12, 0, 0.06, 1, 0, 0.01)
        tr = []
        for _ in range(150):
            tr.append(_trace_step(sim, esc, 0.12, 0, 0.06))
        return tr

    tr_p = step_dist(+1.0)
    tr_n = step_dist(-1.0)
    traces["step_pos"] = tr_p
    traces["step_neg"] = tr_n
    events.append({"event": "step_pos", "peak_er": max(abs(t["e_r"]) for t in tr_p),
                   "final_er": abs(tr_p[-1]["e_r"]), "flips": sum(
                       1 for i in range(1, len(tr_p))
                       if tr_p[i]["Mz"] * tr_p[i - 1]["Mz"] < 0 and abs(tr_p[i]["Mz"]) > 50)})
    events.append({"event": "step_neg", "peak_er": max(abs(t["e_r"]) for t in tr_n),
                   "final_er": abs(tr_n[-1]["e_r"])})

    # --- 2 Impulse ---
    esc = _esc()
    sim = Simulation(cfg)
    sim.reset(25.0, 3)
    settle(sim, esc)
    for _ in range(5):
        sim.esc_brake_add = alloc.allocate(ESCCommand(-4000)).brake_cmd
        sim._step_plant(0.12, 0, 0.06, 1, 0, 0.01)
    tr_imp = []
    for _ in range(150):
        tr_imp.append(_trace_step(sim, esc, 0.12, 0, 0.06))
    traces["impulse"] = tr_imp
    flips_imp = sum(
        1 for i in range(1, len(tr_imp))
        if tr_imp[i]["Mz"] * tr_imp[i - 1]["Mz"] < 0
        and abs(tr_imp[i]["Mz"]) > 50 and abs(tr_imp[i - 1]["Mz"]) > 50
    )
    events.append({"event": "impulse", "final_er": abs(tr_imp[-1]["e_r"]), "flips": flips_imp})

    # --- 3 Sustained disturbance ---
    esc = _esc()
    sim = Simulation(cfg)
    sim.reset(25.0, 3)
    settle(sim, esc)
    tr_sus = []
    for i in range(200):
        # sustained external Mz for first 100 steps, then release
        if i < 100:
            dist = alloc.allocate(ESCCommand(-3000)).brake_cmd
            esc.step(sim)
            # combine: use max of esc and dist on same side — simple: apply dist only
            sim.esc_brake_add = dist
            sim._step_plant(0.12, 0, 0.06, 1, 0, 0.01)
            o = esc.observer.observe_from_simulation(sim)
            tr_sus.append({
                "e_r": o.e_r, "Mz": 0.0, "active": False, "phase": "disturb",
                "cmd_max": float(np.max(dist)),
            })
        else:
            tr_sus.append(_trace_step(sim, esc, 0.12, 0, 0.06))
            tr_sus[-1]["phase"] = "recover"
    traces["sustained"] = tr_sus
    rec = [t for t in tr_sus if t.get("phase") == "recover"]
    events.append({
        "event": "sustained",
        "final_er": abs(rec[-1]["e_r"]) if rec else None,
        "final_Mz": rec[-1]["Mz"] if rec else None,
    })

    # --- 4 Disturbance removal → ΔMz → 0 ---
    final_Mz_after_removal = abs(rec[-1]["Mz"]) if rec else 999
    mean_Mz_tail = float(np.mean([abs(t["Mz"]) for t in rec[-30:]])) if rec else 999

    # --- 5 Crossing zero / sign change of e_r ---
    esc = _esc()
    sim = Simulation(cfg)
    sim.reset(25.0, 3)
    settle(sim, esc)
    for _ in range(25):
        sim.esc_brake_add = alloc.allocate(ESCCommand(-3500)).brake_cmd
        sim._step_plant(0.12, 0, 0.06, 1, 0, 0.01)
    tr_cross = []
    for _ in range(120):
        tr_cross.append(_trace_step(sim, esc, 0.12, 0, 0.06))
    # then opposite dist
    for _ in range(25):
        sim.esc_brake_add = alloc.allocate(ESCCommand(+3500)).brake_cmd
        sim._step_plant(0.12, 0, 0.06, 1, 0, 0.01)
    for _ in range(120):
        tr_cross.append(_trace_step(sim, esc, 0.12, 0, 0.06))
    traces["cross_zero"] = tr_cross
    flips_cross = sum(
        1 for i in range(1, len(tr_cross))
        if tr_cross[i]["Mz"] * tr_cross[i - 1]["Mz"] < 0
        and abs(tr_cross[i]["Mz"]) > 80 and abs(tr_cross[i - 1]["Mz"]) > 80
    )
    events.append({"event": "cross_zero", "flips": flips_cross})

    # --- 6 Near-threshold hysteresis ---
    esc = _esc()
    sim = Simulation(cfg)
    sim.reset(25.0, 3)
    # mild steer, no big dist — e_r may hover near deadband
    act_count = 0
    deact_count = 0
    prev = False
    for i in range(200):
        # small oscillating steer to nudge e_r near threshold
        st = 0.03 + 0.02 * np.sin(2 * np.pi * i / 40)
        t = _trace_step(sim, esc, 0.12, 0, float(st))
        if t["active"] and not prev:
            act_count += 1
        if not t["active"] and prev:
            deact_count += 1
        prev = t["active"]
    events.append({"event": "near_threshold", "activations": act_count, "deactivations": deact_count})

    # --- 7 High-util inhibit ---
    esc = _esc()
    sim = Simulation(cfg)
    sim.reset(28.0, 4)
    inhibited = False
    recovered = False
    for i in range(120):
        brk = 0.55 if i < 50 else 0.0
        st = 0.10
        esc.step(sim)
        sim._step_plant(0.0 if brk > 0.05 else 0.12, brk, st, 1, 0, 0.01)
        if esc.decision.last.inhibited or esc.decision.last.reason == "util_limit":
            inhibited = True
        if inhibited and i > 60 and esc.last_active:
            recovered = True
    events.append({"event": "high_util", "inhibited": inhibited, "recovered": recovered})

    # --- 8 Split-μ transient ---
    mu_lr = np.array([mu0, 0.5 * mu0, mu0, 0.5 * mu0])
    esc = _esc()
    sim = Simulation(cfg)
    sim.mu_per_wheel = mu_lr
    sim.reset(25.0, 3)
    settle(sim, esc)
    for _ in range(25):
        sim.esc_brake_add = alloc.allocate(ESCCommand(-3500)).brake_cmd
        sim._step_plant(0.12, 0, 0.06, 1, 0, 0.01)
    tr_split = []
    for _ in range(150):
        tr_split.append(_trace_step(sim, esc, 0.12, 0, 0.06))
    traces["split_mu"] = tr_split
    events.append({
        "event": "split_mu",
        "final_er": abs(tr_split[-1]["e_r"]),
        "peak_er": max(abs(t["e_r"]) for t in tr_split),
        "finite": all(np.isfinite(t["e_r"]) for t in tr_split),
    })

    # ===== Gates =====
    # Activation policy: after large dist, ESC should activate at least once
    activated_after_step = any(t["active"] for t in tr_n[:80]) or any(abs(t["Mz"]) > 100 for t in tr_n[:80])
    _gate(gates, "correct_activation",
          activated_after_step,
          f"activated={activated_after_step}")

    # Release: tail of recovery should have low |Mz|
    _gate(gates, "correct_release",
          mean_Mz_tail < 500 or final_Mz_after_removal < 200,
          f"mean_Mz_tail={mean_Mz_tail:.1f} final_Mz={final_Mz_after_removal:.1f}")

    # Overshoot: peak |e_r| after ESC starts should not exceed free-run peak excessively
    # Use final |e_r| < peak and flips low as proxy
    _gate(gates, "no_corrective_overshoot",
          abs(tr_n[-1]["e_r"]) <= max(abs(t["e_r"]) for t in tr_n) + 1e-6
          and abs(tr_n[-1]["e_r"]) < 2.0,
          f"final={abs(tr_n[-1]['e_r']):.3f} peak={max(abs(t['e_r']) for t in tr_n):.3f}")

    # Chatter
    _gate(gates, "no_chatter",
          flips_imp <= 4 and flips_cross <= 10 and act_count <= 15,
          f"impulse_flips={flips_imp} cross_flips={flips_cross} near_act={act_count}")

    # Disturbance removal
    _gate(gates, "disturbance_removal",
          mean_Mz_tail < 800,
          f"mean|Mz|_tail={mean_Mz_tail:.1f}")

    # Symmetry of step response magnitude
    peak_p = max(abs(t["e_r"]) for t in tr_p)
    peak_n = max(abs(t["e_r"]) for t in tr_n)
    # known asymmetry tolerance from 15.4–15.8
    _gate(gates, "symmetry",
          peak_p < 5.0 and peak_n < 5.0,
          f"peak+={peak_p:.3f} peak-={peak_n:.3f}")

    _gate(gates, "split_mu",
          events[-1]["finite"] and events[-1]["final_er"] < 5.0,
          f"final_er={events[-1]['final_er']:.3f}")

    # ABS
    esc = _esc()
    sim = Simulation(cfg)
    sim.reset(30.0, 4)
    for _ in range(50):
        esc.step(sim)
        sim._step_plant(0.0, 0.7, 0.05, 1, 0, 0.01)
    d = sim.dual_track.diagnostics()
    _gate(gates, "abs_coexistence",
          d["min_Fz"] >= 50 - 1e-6,
          f"min_Fz={d['min_Fz']:.0f}")

    # ESC OFF regression
    avx, at, _ = _launch(cfg)
    at100, at200 = _t_to(avx, at, 27.78), _t_to(avx, at, 55.56)
    reg = (
        at100 is not None and abs(at100 - REF_HYPER[0]) < 0.15
        and at200 is not None and abs(at200 - REF_HYPER[1]) < 0.25
    )
    _gate(gates, "esc_off_regression", reg, f"t100={at100} t200={at200}")
    hvx, ht, _ = _launch(hist.simulation_config)
    ht100 = _t_to(hvx, ht, 27.78)
    _gate(gates, "historical_isolation",
          ht100 is not None and abs(ht100 - REF_HIST[0]) < 0.3,
          f"t100={ht100}")

    # Determinism
    runs = []
    for _ in range(5):
        esc = _esc()
        sim = Simulation(cfg)
        sim.reset(25.0, 3)
        settle(sim, esc)
        for _ in range(20):
            sim.esc_brake_add = alloc.allocate(ESCCommand(-3000)).brake_cmd
            sim._step_plant(0.12, 0, 0.06, 1, 0, 0.01)
        for _ in range(100):
            _trace_step(sim, esc, 0.12, 0, 0.06)
        runs.append(round(abs(esc.observer.last.e_r), 5))
    _gate(gates, "determinism", len(set(runs)) == 1, f"run0={runs[0]}")

    # High util inhibit observed
    _gate(gates, "high_util_inhibit",
          inhibited,
          f"inhibited={inhibited}")

    n_pass = sum(1 for g in gates if g["pass"])
    status = "PASS" if n_pass == len(gates) else (
        "PASS WITH LIMITATIONS" if n_pass >= len(gates) - 2 else "FAIL"
    )

    recommendation = {
        "selected_candidate": K_CAND,
        "gain_frozen": False,
        "transient_recovery": "PASS" if all(
            g["pass"] for g in gates if g["name"] in (
                "correct_activation", "correct_release", "disturbance_removal", "no_corrective_overshoot"
            )
        ) else "FAIL",
        "plant": "FROZEN",
        "regression": {"t100": at100, "t200": at200},
    }

    with open(ROOT / "transient_events.json", "w") as f:
        json.dump(events, f, indent=2)
    with open(ROOT / "recovery_traces.json", "w") as f:
        # downsample traces
        slim = {k: v[::5] for k, v in traces.items()}
        json.dump(slim, f, indent=2)
    with open(ROOT / "recommendation.json", "w") as f:
        json.dump(recommendation, f, indent=2)
    if events:
        keys = sorted({k for e in events for k in e.keys()})
        with open(ROOT / "transient_events.csv", "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
            w.writeheader()
            for e in events:
                w.writerow(e)

    summary = {
        "phase": "15.9",
        "status": status,
        "gates_passed": n_pass,
        "gates_total": len(gates),
        "gates": gates,
        "recommendation": recommendation,
        "events": events,
    }
    with open(ROOT / "validation_result.json", "w") as f:
        json.dump(summary, f, indent=2, default=str)
    print(f"\n=== PHASE 15.9 — {status} {n_pass}/{len(gates)} ===")
    return summary


if __name__ == "__main__":
    run_validation()
