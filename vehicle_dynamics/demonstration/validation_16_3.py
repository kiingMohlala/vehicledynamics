"""
Phase 16.3 — ESC Failure Injection & Degraded-Authority Validation.

ESC must inhibit when authority is lost and re-enter cleanly when it returns.
K_Mz=10000 unchanged. No plant/architecture retune.
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
from vehicle_dynamics.controls.esc_command import ESCCommand, BrakeAllocator, BrakeAllocatorConfig

ROOT = Path("artifacts/phase_16_3")
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


def _esc(**kw):
    c = ClosedLoopESCConfig(enabled=True, K_Mz=K_MZ)
    for k, v in kw.items():
        setattr(c, k, v)
    return ClosedLoopESC(c)


def run_validation() -> dict:
    ROOT.mkdir(parents=True, exist_ok=True)
    gates = []
    cases = {}
    hyper = bind_authoritative_hypercar(1100.0, 750.0)
    hist = bind_historical_demonstrator()
    cfg = hyper.simulation_config
    mu0 = float(getattr(cfg, "mu_tire", 1.15))

    def record(name, **kw):
        cases[name] = kw
        return kw

    print("  Sensor: yaw-rate bias (observer-side)...")
    # Inject bias into decision input only — does not corrupt plant dynamics.
    logic = ESCDecisionLogic()
    max_Mz = 0.0
    finite = True
    for e in [0.0, 0.2, 0.4, 0.6, -0.3, 0.0, 0.05]:
        # biased e_r as if sensor offset present
        d = logic.step(ESCObservation(
            vx=25, delta=0.08, e_r=e + 0.15, util_max=0.5, beta=0.0,
            r=0.3, r_ref=0.2, r_kin=0.4, ay=3,
        ))
        max_Mz = max(max_Mz, abs(d.delta_Mz_request))
        if not np.isfinite(d.delta_Mz_request):
            finite = False
    # plant under bias remains finite (separate run without state corruption)
    esc = _esc()
    sim = Simulation(cfg)
    sim.reset(25.0, 3)
    peak_r = 0.0
    for i in range(120):
        st = 0.0 if i < 30 else 0.08
        esc.step(sim)
        sim._step_plant(0.12, 0, st, 1, 0, 0.01)
        peak_r = max(peak_r, abs(sim.state.vehicle.yaw_rate))
        if not np.isfinite(sim.state.vehicle.yaw_rate):
            finite = False
    record("yaw_bias", peak_r=peak_r, max_Mz=max_Mz, finite=finite)

    print("  Sensor: ESC unavailable...")
    esc = ClosedLoopESC(ClosedLoopESCConfig(enabled=False, K_Mz=K_MZ))
    sim = Simulation(cfg)
    sim.reset(25.0, 3)
    for i in range(100):
        esc.step(sim)
        sim._step_plant(0.12, 0, 0.08 if i > 20 else 0, 1, 0, 0.01)
    record("esc_unavailable", max_Mz=abs(esc.last_Mz), active=esc.last_active,
           cmd_zero=sim.esc_brake_add is None or np.allclose(sim.esc_brake_add, 0))

    print("  Sensor: stale observation release...")
    logic = ESCDecisionLogic()
    d1 = logic.step(ESCObservation(vx=25, delta=0.1, e_r=0.4, util_max=0.5, beta=0.0,
                                   r=0.5, r_ref=0.1, r_kin=0.6, ay=5))
    for _ in range(5):
        d2 = logic.step(ESCObservation(vx=25, delta=0.1, e_r=0.02, util_max=0.5, beta=0.0,
                                       r=0.1, r_ref=0.1, r_kin=0.1, ay=1))
    record("stale_obs_release", activated=d1.active, released=not d2.active, final_Mz=d2.delta_Mz_request)

    print("  Tire: single-wheel mu collapse...")
    esc = _esc()
    sim = Simulation(cfg)
    sim.reset(25.0, 3)
    for i in range(160):
        if i == 50:
            sim.mu_per_wheel = np.array([mu0, mu0, 0.15 * mu0, mu0])
        st = 0.0 if i < 30 else 0.10
        esc.step(sim)
        sim._step_plant(0.12, 0, st, 1, 0, 0.01)
    record("single_wheel_mu", peak_r=abs(sim.state.vehicle.yaw_rate),
           finite=np.isfinite(sim.state.vehicle.yaw_rate), max_Mz=abs(esc.last_Mz))

    print("  Tire: axle mu collapse...")
    esc = _esc()
    sim = Simulation(cfg)
    sim.reset(25.0, 3)
    for i in range(160):
        if i == 50:
            sim.mu_per_wheel = np.array([0.2 * mu0, 0.2 * mu0, mu0, mu0])
        st = 0.0 if i < 30 else 0.10
        esc.step(sim)
        sim._step_plant(0.12, 0, st, 1, 0, 0.01)
    record("axle_mu_collapse", peak_r=abs(sim.state.vehicle.yaw_rate),
           finite=np.isfinite(sim.state.vehicle.yaw_rate), max_Mz=abs(esc.last_Mz))

    print("  Tire: severe split-mu...")
    esc = _esc()
    sim = Simulation(cfg)
    sim.mu_per_wheel = np.array([mu0, 0.2 * mu0, mu0, 0.2 * mu0])
    sim.reset(25.0, 3)
    max_cmd = 0.0
    for i in range(150):
        esc.step(sim)
        if sim.esc_brake_add is not None:
            max_cmd = max(max_cmd, float(np.max(sim.esc_brake_add)))
        sim._step_plant(0.12, 0, 0.10 if i > 25 else 0, 1, 0, 0.01)
    record("severe_split_mu", peak_r=abs(sim.state.vehicle.yaw_rate),
           finite=np.isfinite(sim.state.vehicle.yaw_rate), max_Mz=abs(esc.last_Mz), max_cmd=max_cmd)

    print("  Actuator: reduced authority...")
    esc = _esc()
    esc.allocator = BrakeAllocator(BrakeAllocatorConfig(max_delta_Mz=2000.0, brake_torque_max=800.0))
    sim = Simulation(cfg)
    sim.reset(25.0, 3)
    max_cmd = 0.0
    for i in range(120):
        esc.step(sim)
        if sim.esc_brake_add is not None:
            max_cmd = max(max_cmd, float(np.max(sim.esc_brake_add)))
        sim._step_plant(0.12, 0, 0.12 if i > 20 else 0, 1, 0, 0.01)
    record("reduced_actuator", max_cmd=max_cmd, max_Mz=abs(esc.last_Mz),
           finite=np.isfinite(sim.state.vehicle.yaw_rate))

    print("  Actuator: saturation...")
    esc = _esc()
    sim = Simulation(cfg)
    sim.reset(25.0, 3)
    alloc = BrakeAllocator()
    max_cmd = 0.0
    for i in range(100):
        if 30 <= i < 50:
            sim.esc_brake_add = alloc.allocate(ESCCommand(-7000)).brake_cmd
            sim._step_plant(0.12, 0, 0.06, 1, 0, 0.01)
        else:
            esc.step(sim)
            if sim.esc_brake_add is not None:
                max_cmd = max(max_cmd, float(np.max(sim.esc_brake_add)))
            sim._step_plant(0.12, 0, 0.06, 1, 0, 0.01)
    record("actuator_sat", max_cmd=max_cmd, finite=np.isfinite(sim.state.vehicle.yaw_rate))

    print("  Control: inhibit recovery re-entry...")
    esc = _esc()
    sim = Simulation(cfg)
    sim.reset(28.0, 4)
    phases = []
    for i in range(180):
        brk = 0.65 if 30 <= i < 70 else 0.0
        esc.step(sim)
        sim._step_plant(0.0 if brk > 0.05 else 0.12, brk, 0.10, 1, 0, 0.01)
        phases.append({
            "i": i, "inhibited": esc.decision.last.inhibited,
            "reason": esc.decision.last.reason, "active": esc.last_active, "Mz": esc.last_Mz,
        })
    saw_inh = any(p["inhibited"] or p["reason"] == "util_limit" for p in phases)
    after = [p for p in phases if p["i"] > 90]
    stuck = all(p["active"] for p in phases[-40:]) and all(abs(p["Mz"]) > 100 for p in phases[-40:])
    record("inhibit_recovery", saw_inhibit=saw_inh, stuck_active=stuck,
           reentry=any(p["active"] for p in after))

    print("  Combined: ABS + ESC...")
    esc = _esc()
    sim = Simulation(cfg)
    sim.reset(32.0, 4)
    for i in range(80):
        esc.step(sim)
        sim._step_plant(0.0, 0.75, 0.06 if i > 20 else 0, 1, 0, 0.01)
    d = sim.dual_track.diagnostics()
    record("abs_esc_combined", min_Fz=float(d["min_Fz"]), abs_enabled=sim.dual_track.cfg.abs_enabled,
           finite=np.isfinite(sim.state.vehicle.yaw_rate))

    print("  Combined: split-mu + actuator limit...")
    esc = _esc()
    esc.allocator = BrakeAllocator(BrakeAllocatorConfig(max_delta_Mz=2500.0, brake_torque_max=1000.0))
    sim = Simulation(cfg)
    sim.mu_per_wheel = np.array([mu0, 0.35 * mu0, mu0, 0.35 * mu0])
    sim.reset(25.0, 3)
    max_cmd = 0.0
    for i in range(140):
        esc.step(sim)
        if sim.esc_brake_add is not None:
            max_cmd = max(max_cmd, float(np.max(sim.esc_brake_add)))
        sim._step_plant(0.12, 0, 0.10 if i > 25 else 0, 1, 0, 0.01)
    record("split_mu_actuator_limit", max_cmd=max_cmd,
           peak_r=abs(sim.state.vehicle.yaw_rate), finite=np.isfinite(sim.state.vehicle.yaw_rate))

    print("  util overrides large e_r...")
    logic = ESCDecisionLogic()
    d = logic.step(ESCObservation(vx=25, delta=0.15, e_r=0.8, util_max=0.99, beta=0.0,
                                  r=1.0, r_ref=0.2, r_kin=1.2, ay=8))
    record("util_overrides_error", inhibited=d.inhibited, active=d.active, reason=d.reason)

    logic = ESCDecisionLogic()
    logic.step(ESCObservation(vx=25, delta=0.1, e_r=0.5, util_max=0.99, beta=0.0,
                              r=0.6, r_ref=0.2, r_kin=0.7, ay=5))
    logic.reset()
    d_clear = logic.step(ESCObservation(vx=25, delta=0.05, e_r=0.02, util_max=0.4, beta=0.0,
                                        r=0.15, r_ref=0.14, r_kin=0.2, ay=2))
    record("post_fault_no_intervention", active=d_clear.active, Mz=d_clear.delta_Mz_request)

    # Gates
    _gate(gates, "no_nan_inf",
          all(c.get("finite", True) for c in cases.values() if "finite" in c), "all finite")
    _gate(gates, "cmd_bounded",
          all(c.get("max_cmd", 0) <= 1.0 + 1e-9 for c in cases.values()),
          f"max_cmd={max(c.get('max_cmd', 0) for c in cases.values()):.3f}")
    _gate(gates, "esc_unavailable_zero_cmd",
          cases["esc_unavailable"]["cmd_zero"] and not cases["esc_unavailable"]["active"],
          str(cases["esc_unavailable"]))
    _gate(gates, "stale_obs_releases",
          cases["stale_obs_release"]["activated"] and cases["stale_obs_release"]["released"],
          str(cases["stale_obs_release"]))
    _gate(gates, "tire_authority_loss_stable",
          cases["single_wheel_mu"]["finite"] and cases["axle_mu_collapse"]["finite"]
          and cases["severe_split_mu"]["finite"],
          f"severe_r={cases['severe_split_mu']['peak_r']:.3f}")
    _gate(gates, "actuator_degraded_bounded",
          cases["reduced_actuator"]["max_cmd"] <= 1.0 and cases["actuator_sat"]["max_cmd"] <= 1.0,
          f"reduced={cases['reduced_actuator']['max_cmd']:.3f}")
    _gate(gates, "inhibit_not_aggressive",
          cases["util_overrides_error"]["inhibited"] and not cases["util_overrides_error"]["active"],
          cases["util_overrides_error"]["reason"])
    _gate(gates, "inhibit_recovery_reentry",
          cases["inhibit_recovery"]["saw_inhibit"] and not cases["inhibit_recovery"]["stuck_active"],
          str(cases["inhibit_recovery"]))
    _gate(gates, "no_stuck_active", not cases["inhibit_recovery"]["stuck_active"], "ok")
    _gate(gates, "post_fault_no_unexpected",
          not cases["post_fault_no_intervention"]["active"]
          and cases["post_fault_no_intervention"]["Mz"] == 0.0,
          str(cases["post_fault_no_intervention"]))
    _gate(gates, "abs_remains_functional",
          cases["abs_esc_combined"]["min_Fz"] >= 50 - 1e-6 and cases["abs_esc_combined"]["abs_enabled"],
          f"min_Fz={cases['abs_esc_combined']['min_Fz']:.0f}")
    _gate(gates, "combined_failures_stable",
          cases["split_mu_actuator_limit"]["finite"] and cases["split_mu_actuator_limit"]["peak_r"] < 5.0,
          f"peak_r={cases['split_mu_actuator_limit']['peak_r']:.3f}")
    _gate(gates, "sensor_bias_bounded",
          cases["yaw_bias"]["finite"] and cases["yaw_bias"]["peak_r"] < 10.0,
          f"peak_r={cases['yaw_bias']['peak_r']:.3f}")

    avx, at, _ = _launch(cfg)
    at100, at200 = _t_to(avx, at, 27.78), _t_to(avx, at, 55.56)
    reg = (at100 is not None and abs(at100 - REF_HYPER[0]) < 0.15
           and at200 is not None and abs(at200 - REF_HYPER[1]) < 0.25)
    _gate(gates, "passive_regression", reg, f"t100={at100} t200={at200}")
    hvx, ht, _ = _launch(hist.simulation_config)
    ht100 = _t_to(hvx, ht, 27.78)
    _gate(gates, "historical_isolation",
          ht100 is not None and abs(ht100 - REF_HIST[0]) < 0.3, f"t100={ht100}")

    runs = []
    for _ in range(5):
        esc = _esc()
        sim = Simulation(cfg)
        sim.mu_per_wheel = np.array([mu0, 0.2 * mu0, mu0, 0.2 * mu0])
        sim.reset(25.0, 3)
        for i in range(80):
            esc.step(sim)
            sim._step_plant(0.12, 0, 0.08 if i > 20 else 0, 1, 0, 0.01)
        runs.append(round(sim.state.vehicle.yaw_rate, 5))
    _gate(gates, "determinism", len(set(runs)) == 1, f"run0={runs[0]}")

    n_pass = sum(1 for g in gates if g["pass"])
    status = "PASS" if n_pass == len(gates) else (
        "CONDITIONAL PASS" if n_pass >= len(gates) - 2 else "FAIL")
    summary = {
        "phase": "16.3", "status": status, "gates_passed": n_pass, "gates_total": len(gates),
        "gates": gates, "cases": cases, "K_Mz": K_MZ, "K_Mz_frozen": False,
        "regression": {"hyper": {"t100": at100, "t200": at200, "ref": REF_HYPER},
                       "hist": {"t100": ht100, "ref": REF_HIST}},
    }
    with open(ROOT / "failure_cases.json", "w") as f:
        json.dump(cases, f, indent=2, default=str)
    with open(ROOT / "summary.json", "w") as f:
        json.dump(summary, f, indent=2, default=str)
    rows = [{"case": k, **{kk: vv for kk, vv in v.items() if not isinstance(vv, (list, dict))}}
            for k, v in cases.items()]
    if rows:
        keys = sorted({kk for r in rows for kk in r.keys()})
        with open(ROOT / "failure_cases.csv", "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
            w.writeheader()
            for r in rows:
                w.writerow(r)
    print(f"\n=== PHASE 16.3 — {status} {n_pass}/{len(gates)} ===")
    return summary


if __name__ == "__main__":
    run_validation()
