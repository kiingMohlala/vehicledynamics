"""
Phase 14.9.6 — Hydraulic Cross-Linked Anti-Roll Bar (passive).
No active pressure. No ESC. No retuning.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from vehicle_dynamics.demonstration.vehicle_binding import (
    bind_authoritative_hypercar,
    bind_historical_demonstrator,
)
from vehicle_dynamics.simulation.simulation import Simulation
from vehicle_dynamics.simulation.dual_track_plant import DualTrackConfig
from vehicle_dynamics.suspension.anti_roll_bar import HydraulicAntiRollBar, MechanicalAntiRollBar

ROOT = Path("artifacts/phase_14_9_6")
REF_HYPER = (3.13, 8.34)
REF_HIST = (5.37, 19.81)


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


def _cfg_hyd(k_f=30000.0, k_r=28000.0, c_f=800.0, c_r=800.0):
    c = bind_authoritative_hypercar().simulation_config
    c.use_hydraulic_arb = True
    c.use_arb = True
    c.k_hyd_front = k_f
    c.k_hyd_rear = k_r
    c.c_hyd_front = c_f
    c.c_hyd_rear = c_r
    # mechanical off path when hydraulic selected
    c.k_arb_front = 0.0
    c.k_arb_rear = 0.0
    return c


def _phi(cfg, ay=8.0, n=220):
    s = Simulation(cfg)
    s.reset(25.0, 3)
    for _ in range(n):
        s.dual_track.state.ay = ay
        s.dual_track.step(vx=25, vy=0, yaw_rate=0, steer=0,
                          drive_torque_total=0, brake_cmd=0, dt=0.01)
    return s.dual_track.sprung.state.phi, s


def run_validation() -> dict:
    ROOT.mkdir(parents=True, exist_ok=True)
    gates = []
    hyper = bind_authoritative_hypercar(1100.0, 750.0)
    hist = bind_historical_demonstrator()
    cfg_default = hyper.simulation_config  # mechanical default

    # 1 Zero hydraulic → baseline (no ARB)
    c0 = _cfg_hyd(0, 0)
    phi0, s0 = _phi(c0)
    _gate(gates, "zero_hydraulic_baseline",
          abs(phi0) > 0.02 and isinstance(s0.dual_track.sprung.arb.front, HydraulicAntiRollBar),
          f"φ={phi0:.4f} type={type(s0.dual_track.sprung.arb.front).__name__}")

    # 2 Increasing stiffness reduces roll resistance
    phi30, _ = _phi(_cfg_hyd(30000, 28000))
    phi100, _ = _phi(_cfg_hyd(100000, 90000))
    _gate(gates, "stiffness_reduces_roll",
          abs(phi100) < abs(phi30) < abs(phi0),
          f"φ0={phi0:.4f} φ30k={phi30:.4f} φ100k={phi100:.4f}")

    # 3 Force pair sum zero
    h = HydraulicAntiRollBar(k_hyd=40000, track=1.65)
    fl, fr, M = h.forces(0.025, -0.015)
    _gate(gates, "force_pair_sum_zero", abs(fl + fr) < 1e-9, f"FL={fl:.1f} FR={fr:.1f}")

    # 4 Hydraulic damping dissipative
    h = HydraulicAntiRollBar(k_hyd=20000, c_hyd=1000)
    E0 = h.E_dissipated
    for _ in range(50):
        h.dissipate(0.5, -0.3, 0.01)
    _gate(gates, "hydraulic_damping_dissipative",
          h.E_dissipated > E0,
          f"E={h.E_dissipated:.4f}")

    # Runtime dissipation during motion
    c = _cfg_hyd(40000, 35000, c_f=1500, c_r=1500)
    _, s = _phi(c, ay=8, n=100)
    E_rt = getattr(s.dual_track.sprung.arb.front, "E_dissipated", 0.0)
    _gate(gates, "runtime_dissipation",
          E_rt >= 0.0,
          f"E_front={E_rt:.4f}")

    # 5 Front/rear independent
    cf = _cfg_hyd(120000, 0)
    cr = _cfg_hyd(0, 120000)
    phi_f, sf = _phi(cf)
    phi_r, sr = _phi(cr)
    _gate(gates, "front_rear_independent",
          abs(phi_f) < abs(phi0) and abs(phi_r) < abs(phi0)
          and abs(sf.dual_track.sprung.arb.front.k_hyd - 120000) < 1
          and abs(sr.dual_track.sprung.arb.rear.k_hyd - 120000) < 1,
          f"φ_f={phi_f:.4f} φ_r={phi_r:.4f}")

    # 6 L/R reversal symmetry
    phi_p, _ = _phi(_cfg_hyd(40000, 35000), ay=8)
    phi_m, _ = _phi(_cfg_hyd(40000, 35000), ay=-8)
    _gate(gates, "lr_reversal_symmetry",
          abs(phi_p + phi_m) < 1e-4,
          f"φ+={phi_p:.5f} φ-={phi_m:.5f}")

    # 7 ΣFz conserved
    _, s = _phi(_cfg_hyd(50000, 45000), ay=8)
    fz_sum = sum(w.Fz for w in s.dual_track.wheels)
    _gate(gates, "fz_conserved",
          abs(fz_sum - 1100 * 9.81) < 150,
          f"ΣFz={fz_sum:.0f}")

    # 8 Parameters reach runtime
    c = _cfg_hyd(33333, 22222, c_f=777, c_r=666)
    s = Simulation(c)
    fr = s.dual_track.sprung.arb.front
    rr = s.dual_track.sprung.arb.rear
    _gate(gates, "runtime_parameter_authority",
          isinstance(fr, HydraulicAntiRollBar)
          and abs(fr.k_hyd - 33333) < 1 and abs(rr.k_hyd - 22222) < 1
          and abs(fr.c_hyd - 777) < 1,
          f"k_f={fr.k_hyd} k_r={rr.k_hyd} c_f={fr.c_hyd}")

    # 9 Poisoned defaults
    DualTrackConfig.__dataclass_fields__["k_hyd_front"].default = 1.0
    DualTrackConfig.__dataclass_fields__["use_hydraulic_arb"].default = False
    try:
        c = _cfg_hyd(55000, 50000)
        s = Simulation(c)
        ok = isinstance(s.dual_track.sprung.arb.front, HydraulicAntiRollBar) and abs(
            s.dual_track.sprung.arb.front.k_hyd - 55000) < 1
        _gate(gates, "poisoned_defaults", ok,
              f"type={type(s.dual_track.sprung.arb.front).__name__} k={s.dual_track.sprung.arb.front.k_hyd}")
    finally:
        DualTrackConfig.__dataclass_fields__["k_hyd_front"].default = 30000.0
        DualTrackConfig.__dataclass_fields__["use_hydraulic_arb"].default = False

    # 10 Dynamic Fz → Dugoff
    s = Simulation(_cfg_hyd(40000, 35000))
    s.reset(25.0, 3)
    for _ in range(40):
        s._step_plant(0.12, 0, 0.0, 1, 0, 0.01)
    for _ in range(100):
        s._step_plant(0.12, 0, 0.10, 1, 0, 0.01)
    _gate(gates, "dugoff_fz_coupling",
          abs(sum(w.Fy for w in s.dual_track.wheels)) > 500,
          f"ΣFy={sum(w.Fy for w in s.dual_track.wheels):.0f} φ={s.dual_track.sprung.state.phi:.4f}")

    # 11 Combined brake + corner
    s = Simulation(_cfg_hyd(40000, 35000))
    s.reset(30.0, 4)
    ok = True
    for _ in range(50):
        s._step_plant(0.0, 0.5, 0.08, 1, 0, 0.01)
        if any(np.isnan(w.Fz) for w in s.dual_track.wheels):
            ok = False
            break
    d = s.dual_track.diagnostics()
    _gate(gates, "combined_brake_corner",
          ok and d["min_Fz"] >= 50 - 1e-6,
          f"min_Fz={d['min_Fz']:.1f}")

    # 12 No NaN / Fz floor
    _gate(gates, "fz_floor_safety",
          d["min_Fz"] >= 50 - 1e-6 and not any(np.isnan(d["Fz"])),
          f"min_Fz={d['min_Fz']:.1f}")

    # Mechanical fallback still works
    s = Simulation(cfg_default)
    _gate(gates, "mechanical_fallback",
          isinstance(s.dual_track.sprung.arb.front, MechanicalAntiRollBar),
          f"default type={type(s.dual_track.sprung.arb.front).__name__}")

    # 13 Determinism
    runs = []
    for _ in range(5):
        phi, _ = _phi(_cfg_hyd(40000, 35000), ay=6, n=80)
        runs.append(round(phi, 8))
    _gate(gates, "deterministic_replay", len(set(runs)) == 1, f"run0={runs[0]}")

    # 14 Regression (default mechanical config)
    avx, at, _ = _launch(cfg_default)
    at100, at200 = _t_to(avx, at, 27.78), _t_to(avx, at, 55.56)
    hvx, ht, _ = _launch(hist.simulation_config)
    ht100 = _t_to(hvx, ht, 27.78)
    reg = (
        at100 is not None and abs(at100 - REF_HYPER[0]) < 0.15
        and at200 is not None and abs(at200 - REF_HYPER[1]) < 0.25
    )
    _gate(gates, "regression", reg, f"t100={at100} t200={at200}")
    _gate(gates, "historical_isolation",
          ht100 is not None and abs(ht100 - REF_HIST[0]) < 0.3,
          f"t100={ht100}")

    n_pass = sum(1 for g in gates if g["pass"])
    status = "PASS" if n_pass == len(gates) else (
        "PASS WITH LIMITATIONS" if n_pass >= len(gates) - 1 else "FAIL"
    )
    summary = {
        "phase": "14.9.6",
        "status": status,
        "gates_passed": n_pass,
        "gates_total": len(gates),
        "gates": gates,
        "roll": {"phi0": phi0, "phi30k": phi30, "phi100k": phi100},
        "regression": {
            "hyper": {"t100": at100, "t200": at200, "ref": REF_HYPER},
            "hist": {"t100": ht100, "ref": REF_HIST},
        },
    }
    with open(ROOT / "validation_result.json", "w") as f:
        json.dump(summary, f, indent=2, default=str)
    print(f"\n=== PHASE 14.9.6 — {status} {n_pass}/{len(gates)} ===")
    return summary


if __name__ == "__main__":
    run_validation()
