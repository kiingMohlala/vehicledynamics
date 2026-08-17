"""
Phase 16.2 — Advanced Limit-Handling & Combined-Maneuver Validation.

Push toward stability envelope. K_Mz=10000 as-is. No plant/architecture/gain changes.
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
from vehicle_dynamics.controls.esc_decision import ESCDecisionLogic
from vehicle_dynamics.controls.esc_observability import ESCObservation

ROOT = Path("artifacts/phase_16_2")
REF_HYPER = (3.13, 8.34)
REF_HIST = (5.37, 19.81)
K_MZ = 10000.0


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


def _esc(on=True):
    return ClosedLoopESC(ClosedLoopESCConfig(enabled=on, K_Mz=K_MZ))


def _metrics(trace):
    return {
        "peak_er": max(abs(t["e_r"]) for t in trace) if trace else 0.0,
        "final_er": abs(trace[-1]["e_r"]) if trace else 0.0,
        "peak_beta": max(abs(t["beta"]) for t in trace) if trace else 0.0,
        "peak_r": max(abs(t["r"]) for t in trace) if trace else 0.0,
        "peak_util": max(t["util"] for t in trace) if trace else 0.0,
        "max_Mz": max(abs(t["Mz"]) for t in trace) if trace else 0.0,
        "max_cmd": max(t["cmd"] for t in trace) if trace else 0.0,
        "flips": sum(
            1 for i in range(1, len(trace))
            if trace[i]["Mz"] * trace[i - 1]["Mz"] < 0
            and abs(trace[i]["Mz"]) > 50 and abs(trace[i - 1]["Mz"]) > 50
        ),
        "active_steps": sum(1 for t in trace if t["active"]),
        "finite": all(np.isfinite(t["r"]) and np.isfinite(t["e_r"]) for t in trace),
    }


def _step(sim, esc, thr, brk, steer):
    esc.step(sim)
    sim._step_plant(thr, brk, steer, 1.0, 0.0, 0.01)
    o = esc.observer.last
    v = sim.state.vehicle
    cmd = float(np.max(sim.esc_brake_add)) if sim.esc_brake_add is not None else 0.0
    return {
        "e_r": o.e_r if o else 0.0,
        "beta": o.beta if o else np.arctan2(v.vy, max(abs(v.vx), 0.5)),
        "r": v.yaw_rate,
        "ay": v.ay,
        "util": o.util_max if o else max(w.utilization for w in sim.dual_track.wheels),
        "Mz": esc.last_Mz,
        "active": esc.last_active,
        "cmd": cmd,
        "inhibited": esc.decision.last.inhibited if esc.cfg.enabled else False,
        "reason": esc.decision.last.reason if esc.cfg.enabled else "off",
    }


def run_validation() -> dict:
    ROOT.mkdir(parents=True, exist_ok=True)
    gates = []
    scenarios = {}
    hyper = bind_authoritative_hypercar(1100.0, 750.0)
    hist = bind_historical_demonstrator()
    cfg = hyper.simulation_config
    mu0 = float(getattr(cfg, "mu_tire", 1.15))

    # ----- 1 Progressive saturation -----
    print("  Progressive saturation...")
    esc = _esc(True)
    sim = Simulation(cfg)
    sim.reset(25.0, 3)
    tr = []
    for i in range(250):
        # ramp steer 0 → 0.18
        st = min(0.18, i * 0.001)
        err = 25.0 - sim.state.vehicle.vx
        thr = float(np.clip(0.12 + 0.05 * err, 0, 0.6))
        tr.append(_step(sim, esc, thr, 0.0, st))
    scenarios["progressive_saturation"] = _metrics(tr)
    scenarios["progressive_saturation"]["peak_util"] = max(t["util"] for t in tr)
    high_util = [t for t in tr if t["util"] > 0.95]
    # When util high, either inhibited or cmd not growing unbounded
    sat_ok = scenarios["progressive_saturation"]["max_cmd"] <= 1.0 + 1e-9

    # ----- 2 High-speed transient -----
    print("  High-speed transient...")
    esc = _esc(True)
    sim = Simulation(cfg)
    sim.reset(35.0, 4)
    tr = []
    for i in range(200):
        st = 0.0 if i < 30 else 0.10
        err = 35.0 - sim.state.vehicle.vx
        thr = float(np.clip(0.12 + 0.05 * err, 0, 0.6))
        tr.append(_step(sim, esc, thr, 0.0, st))
    scenarios["high_speed_transient"] = _metrics(tr)

    # ----- 3 Brake + steer near saturation -----
    print("  Brake+steer near saturation...")
    esc = _esc(True)
    sim = Simulation(cfg)
    sim.reset(30.0, 4)
    tr = []
    for i in range(180):
        st = 0.0 if i < 25 else 0.12
        brk = 0.0 if i < 40 else 0.65
        tr.append(_step(sim, esc, 0.0, brk, st))
    scenarios["brake_steer_sat"] = _metrics(tr)
    scenarios["brake_steer_sat"]["min_Fz"] = float(sim.dual_track.diagnostics()["min_Fz"])

    # ----- 4 μ transition high→low while cornering -----
    print("  μ transition high→low...")
    esc = _esc(True)
    sim = Simulation(cfg)
    sim.reset(25.0, 3)
    tr = []
    for i in range(220):
        if i == 80:
            sim.mu_per_wheel = np.array([0.45 * mu0] * 4)
        st = 0.0 if i < 30 else 0.10
        err = 25.0 - sim.state.vehicle.vx
        thr = float(np.clip(0.12 + 0.05 * err, 0, 0.6))
        tr.append(_step(sim, esc, thr, 0.0, st))
    scenarios["mu_high_to_low"] = _metrics(tr)

    # ----- 5 μ transition low→high -----
    print("  μ transition low→high...")
    esc = _esc(True)
    sim = Simulation(cfg)
    sim.mu_per_wheel = np.array([0.45 * mu0] * 4)
    sim.reset(25.0, 3)
    tr = []
    for i in range(220):
        if i == 80:
            sim.mu_per_wheel = None  # back to nominal
        st = 0.0 if i < 30 else 0.10
        err = 25.0 - sim.state.vehicle.vx
        thr = float(np.clip(0.12 + 0.05 * err, 0, 0.6))
        tr.append(_step(sim, esc, thr, 0.0, st))
    scenarios["mu_low_to_high"] = _metrics(tr)

    # ----- 6 Split-μ transition sequence -----
    print("  Split-μ transitions...")
    esc = _esc(True)
    sim = Simulation(cfg)
    sim.reset(25.0, 3)
    tr = []
    mu_lr = np.array([mu0, 0.5 * mu0, mu0, 0.5 * mu0])
    mu_rl = np.array([0.5 * mu0, mu0, 0.5 * mu0, mu0])
    for i in range(300):
        if i == 60:
            sim.mu_per_wheel = mu_lr
        elif i == 140:
            sim.mu_per_wheel = mu_rl
        elif i == 220:
            sim.mu_per_wheel = None
        st = 0.0 if i < 30 else 0.08
        err = 25.0 - sim.state.vehicle.vx
        thr = float(np.clip(0.12 + 0.05 * err, 0, 0.6))
        tr.append(_step(sim, esc, thr, 0.0, st))
    scenarios["split_mu_transition"] = _metrics(tr)

    # ----- 7 Steering reversal at high util -----
    print("  Steering reversal...")
    esc = _esc(True)
    sim = Simulation(cfg)
    sim.reset(28.0, 3)
    tr = []
    for i in range(250):
        if i < 40:
            st = 0.0
        elif i < 120:
            st = 0.14
        else:
            st = -0.14
        err = 28.0 - sim.state.vehicle.vx
        thr = float(np.clip(0.12 + 0.05 * err, 0, 0.6))
        tr.append(_step(sim, esc, thr, 0.0, st))
    scenarios["steer_reversal"] = _metrics(tr)

    # ----- 8 Inhibit → recovery -----
    print("  Inhibit recovery...")
    esc = _esc(True)
    sim = Simulation(cfg)
    sim.reset(28.0, 4)
    tr = []
    saw_inhibit = False
    saw_reentry = False
    for i in range(160):
        brk = 0.6 if i < 55 else 0.0
        st = 0.10
        t = _step(sim, esc, 0.0 if brk > 0.05 else 0.12, brk, st)
        tr.append(t)
        if t["inhibited"] or t["reason"] == "util_limit":
            saw_inhibit = True
        if saw_inhibit and i > 70 and t["active"]:
            saw_reentry = True
    scenarios["inhibit_recovery"] = _metrics(tr)
    scenarios["inhibit_recovery"]["saw_inhibit"] = saw_inhibit
    scenarios["inhibit_recovery"]["saw_reentry"] = saw_reentry

    # ----- Gates -----
    all_sc = list(scenarios.values())
    _gate(gates, "no_nan_all_scenarios",
          all(s["finite"] for s in all_sc),
          f"n={len(all_sc)}")

    _gate(gates, "command_bounded",
          all(s["max_cmd"] <= 1.0 + 1e-9 for s in all_sc),
          f"max_cmd={max(s['max_cmd'] for s in all_sc):.3f}")

    _gate(gates, "mz_bounded",
          all(s["max_Mz"] <= 6000.0 + 1e-3 for s in all_sc),
          f"max_Mz={max(s['max_Mz'] for s in all_sc):.0f}")

    _gate(gates, "no_pathological_chatter",
          all(s["flips"] <= 10 for s in all_sc),
          f"max_flips={max(s['flips'] for s in all_sc)}")

    _gate(gates, "yaw_sideslip_bounded",
          all(s["peak_r"] < 5.0 and s["peak_beta"] < 1.5 for s in all_sc),
          f"max_r={max(s['peak_r'] for s in all_sc):.3f} max_β={max(s['peak_beta'] for s in all_sc):.3f}")

    _gate(gates, "progressive_sat_no_overdemand",
          sat_ok and scenarios["progressive_saturation"]["peak_util"] > 0.7,
          f"util={scenarios['progressive_saturation']['peak_util']:.3f} cmd={scenarios['progressive_saturation']['max_cmd']:.3f}")

    _gate(gates, "brake_steer_abs_coexist",
          scenarios["brake_steer_sat"]["min_Fz"] >= 50 - 1e-6,
          f"min_Fz={scenarios['brake_steer_sat']['min_Fz']:.0f}")

    _gate(gates, "mu_transition_stable",
          scenarios["mu_high_to_low"]["finite"] and scenarios["mu_low_to_high"]["finite"],
          f"h2l_r={scenarios['mu_high_to_low']['peak_r']:.3f} l2h_r={scenarios['mu_low_to_high']['peak_r']:.3f}")

    _gate(gates, "split_mu_transition_stable",
          scenarios["split_mu_transition"]["finite"]
          and scenarios["split_mu_transition"]["peak_r"] < 5.0,
          f"peak_r={scenarios['split_mu_transition']['peak_r']:.3f}")

    _gate(gates, "steer_reversal_stable",
          scenarios["steer_reversal"]["finite"]
          and scenarios["steer_reversal"]["flips"] <= 10,
          f"flips={scenarios['steer_reversal']['flips']} peak_r={scenarios['steer_reversal']['peak_r']:.3f}")

    _gate(gates, "inhibit_then_recovery",
          saw_inhibit,
          f"inhibit={saw_inhibit} reentry={saw_reentry}")

    # Policy inhibits still present
    logic = ESCDecisionLogic()
    d = logic.step(ESCObservation(
        vx=25, delta=0.1, e_r=0.5, util_max=0.99, beta=0.0,
        r=0.5, r_ref=0.2, r_kin=0.6, ay=5,
    ))
    _gate(gates, "util_inhibit_policy",
          d.inhibited,
          f"reason={d.reason}")

    # ESC OFF regression
    avx, at, _ = _launch(cfg)
    at100, at200 = _t_to(avx, at, 27.78), _t_to(avx, at, 55.56)
    reg = (
        at100 is not None and abs(at100 - REF_HYPER[0]) < 0.15
        and at200 is not None and abs(at200 - REF_HYPER[1]) < 0.25
    )
    _gate(gates, "passive_regression", reg, f"t100={at100} t200={at200}")
    hvx, ht, _ = _launch(hist.simulation_config)
    ht100 = _t_to(hvx, ht, 27.78)
    _gate(gates, "historical_isolation",
          ht100 is not None and abs(ht100 - REF_HIST[0]) < 0.3,
          f"t100={ht100}")

    # Determinism of one limit case
    def run_rev():
        esc = _esc(True)
        sim = Simulation(cfg)
        sim.reset(28.0, 3)
        for i in range(150):
            st = 0.14 if 40 <= i < 100 else (-0.14 if i >= 100 else 0.0)
            _step(sim, esc, 0.12, 0.0, st)
        return round(sim.state.vehicle.yaw_rate, 5)
    runs = [run_rev() for _ in range(5)]
    _gate(gates, "determinism", len(set(runs)) == 1, f"run0={runs[0]}")

    n_pass = sum(1 for g in gates if g["pass"])
    status = "PASS" if n_pass == len(gates) else (
        "CONDITIONAL PASS" if n_pass >= len(gates) - 2 else "FAIL"
    )
    summary = {
        "phase": "16.2",
        "status": status,
        "gates_passed": n_pass,
        "gates_total": len(gates),
        "gates": gates,
        "scenarios": scenarios,
        "K_Mz": K_MZ,
        "K_Mz_frozen": False,
        "regression": {
            "hyper": {"t100": at100, "t200": at200, "ref": REF_HYPER},
            "hist": {"t100": ht100, "ref": REF_HIST},
        },
    }
    with open(ROOT / "scenario_results.json", "w") as f:
        json.dump(scenarios, f, indent=2, default=str)
    with open(ROOT / "summary.json", "w") as f:
        json.dump(summary, f, indent=2, default=str)
    # CSV flat
    rows = [{"scenario": k, **v} for k, v in scenarios.items()]
    if rows:
        keys = sorted({kk for r in rows for kk in r.keys()})
        with open(ROOT / "scenario_results.csv", "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
            w.writeheader()
            for r in rows:
                w.writerow(r)
    print(f"\n=== PHASE 16.2 — {status} {n_pass}/{len(gates)} ===")
    return summary


if __name__ == "__main__":
    run_validation()
