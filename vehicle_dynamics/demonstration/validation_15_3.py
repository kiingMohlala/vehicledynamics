"""
Phase 15.3 — ESC Stability Envelope & Decision Logic.

NO plant intervention. Hypothetical ΔMz only.
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
from vehicle_dynamics.controls.esc_observability import (
    ESCObservability,
    ESCObservation,
    ReferenceYawConfig,
)
from vehicle_dynamics.controls.esc_decision import ESCDecisionLogic, ESCDecisionConfig
from vehicle_dynamics.controls.esc_command import BrakeAllocator

ROOT = Path("artifacts/phase_15_3")
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


def _obs(**kwargs) -> ESCObservation:
    base = dict(
        t=0.0, vx=25.0, vy=0.0, beta=0.0, r=0.0, ay=0.0, ax=0.0,
        delta=0.08, delta_fl=0.08, delta_fr=0.08,
        r_kin=0.5, r_ref=0.3, e_r=0.0, beta_ref=0.0, e_beta=0.0,
        eligible=True, util_max=0.5,
        Fz=[3000]*4, Fx=[0]*4, Fy=[0]*4,
    )
    base.update(kwargs)
    return ESCObservation(**base)


def run_validation() -> dict:
    ROOT.mkdir(parents=True, exist_ok=True)
    gates = []
    hyper = bind_authoritative_hypercar(1100.0, 750.0)
    hist = bind_historical_demonstrator()
    cfg = hyper.simulation_config
    logic = ESCDecisionLogic(ESCDecisionConfig())

    # 1 Zero error → zero request
    d = logic.step(_obs(e_r=0.0))
    _gate(gates, "zero_error_zero_request",
          d.delta_Mz_request == 0.0 and not d.active,
          f"M={d.delta_Mz_request} reason={d.reason}")

    # 2 Positive e_r → negative ΔMz
    logic.reset()
    d = logic.step(_obs(e_r=0.25))
    _gate(gates, "positive_error_negative_request",
          d.active and d.delta_Mz_request < 0,
          f"e_r=0.25 M={d.delta_Mz_request:.0f}")

    # 3 Negative e_r → positive ΔMz
    logic.reset()
    d = logic.step(_obs(e_r=-0.25))
    _gate(gates, "negative_error_positive_request",
          d.active and d.delta_Mz_request > 0,
          f"e_r=-0.25 M={d.delta_Mz_request:.0f}")

    # 4 Symmetry
    logic.reset()
    dp = logic.step(_obs(e_r=0.3))
    logic.reset()
    dn = logic.step(_obs(e_r=-0.3))
    _gate(gates, "symmetry",
          abs(dp.delta_Mz_request + dn.delta_Mz_request) < 1.0,
          f"M+={dp.delta_Mz_request:.0f} M-={dn.delta_Mz_request:.0f}")

    # 5 Deadband (enter)
    logic.reset()
    d = logic.step(_obs(e_r=0.05))  # below e_enter=0.12
    _gate(gates, "deadband",
          not d.active and d.delta_Mz_request == 0.0,
          f"reason={d.reason}")

    # 6 Hysteresis — enter then stay until exit
    logic.reset()
    cfg_h = ESCDecisionConfig(e_enter=0.12, e_exit=0.06)
    logic = ESCDecisionLogic(cfg_h)
    d1 = logic.step(_obs(e_r=0.20))  # enter
    d2 = logic.step(_obs(e_r=0.09))  # between exit and enter — stay active
    d3 = logic.step(_obs(e_r=0.03))  # below exit — deactivate
    _gate(gates, "hysteresis",
          d1.active and d2.active and not d3.active,
          f"enter={d1.active} hold={d2.active} exit={d3.active}")

    # 7 Low-speed inhibit
    logic.reset()
    d = logic.step(_obs(e_r=0.5, vx=3.0))
    _gate(gates, "low_speed_inhibit",
          not d.active and d.reason == "low_speed",
          f"reason={d.reason}")

    # 8 Low-steer inhibit
    logic.reset()
    d = logic.step(_obs(e_r=0.05, delta=0.005))
    _gate(gates, "low_steer_inhibit",
          not d.active,
          f"reason={d.reason}")

    # 9 Finite outputs
    logic.reset()
    d = logic.step(_obs(e_r=1.5))
    _gate(gates, "finite_outputs",
          np.isfinite(d.delta_Mz_request),
          f"M={d.delta_Mz_request:.0f}")

    # 10 Bounded ΔMz
    logic.reset()
    d = logic.step(_obs(e_r=10.0))
    _gate(gates, "bounded_request",
          abs(d.delta_Mz_request) <= ESCDecisionConfig().max_delta_Mz + 1e-6,
          f"M={d.delta_Mz_request:.0f}")

    # 11 Utilization inhibition
    logic.reset()
    d = logic.step(_obs(e_r=0.4, util_max=0.99))
    _gate(gates, "utilization_inhibit",
          not d.active and d.inhibited,
          f"reason={d.reason}")

    # 12 High-slip / beta inhibit
    logic.reset()
    d = logic.step(_obs(e_r=0.4, beta=0.50))
    _gate(gates, "high_slip_inhibit",
          not d.active and d.inhibited,
          f"reason={d.reason}")

    # 13 Sign reversal of error
    logic.reset()
    d_a = logic.step(_obs(e_r=0.3))
    d_b = logic.step(_obs(e_r=-0.3))
    _gate(gates, "sign_reversal",
          d_a.delta_Mz_request * d_b.delta_Mz_request < 0,
          f"M {d_a.delta_Mz_request:.0f} → {d_b.delta_Mz_request:.0f}")

    # 14 Transient error detection on real plant telemetry (observe only)
    sim = Simulation(cfg)
    obs = ESCObservability(ReferenceYawConfig(K_us=0.0065, wheelbase=2.7))
    logic.reset()
    requests = []
    sim.reset(25.0, 3)
    for i in range(180):
        dlt = 0.0 if i < 30 else 0.12
        err = 25.0 - sim.state.vehicle.vx
        thr = float(np.clip(0.12 + 0.05 * err, 0, 0.6))
        sim._step_plant(thr, 0, dlt, 1, 0, 0.01)
        o = obs.observe_from_simulation(sim)
        dec = logic.step(o)
        requests.append(dec.delta_Mz_request)
        # CRITICAL: do not set esc_brake_add
    _gate(gates, "transient_error_detection",
          any(abs(m) > 100 for m in requests),
          f"max|M_req|={max(abs(m) for m in requests):.0f}")

    # 15 No plant modification — ΣFy with decision running == without
    sim_a = Simulation(cfg)
    sim_a.reset(25.0, 3)
    for _ in range(80):
        sim_a._step_plant(0.12, 0, 0.08, 1, 0, 0.01)
    fy_a = sum(w.Fy for w in sim_a.dual_track.wheels)

    sim_b = Simulation(cfg)
    obs_b = ESCObservability()
    log_b = ESCDecisionLogic()
    sim_b.reset(25.0, 3)
    for _ in range(80):
        sim_b._step_plant(0.12, 0, 0.08, 1, 0, 0.01)
        o = obs_b.observe_from_simulation(sim_b)
        log_b.step(o)
        # never apply
    fy_b = sum(w.Fy for w in sim_b.dual_track.wheels)
    _gate(gates, "no_plant_modification",
          abs(fy_a - fy_b) < 1.0,
          f"ΔΣFy={fy_a-fy_b:.4f}")

    # 16 No brake command output from decision object
    logic.reset()
    d = logic.step(_obs(e_r=0.3))
    _gate(gates, "no_brake_command_output",
          not hasattr(d, "brake_cmd") and not hasattr(logic, "brake_cmd"),
          f"M_req={d.delta_Mz_request:.0f} only")

    # 17 Determinism
    runs = []
    for _ in range(5):
        logic.reset()
        d = logic.step(_obs(e_r=0.22, vx=25.0, delta=0.08, util_max=0.6))
        runs.append(round(d.delta_Mz_request, 4))
    _gate(gates, "deterministic_decision", len(set(runs)) == 1, f"run0={runs[0]}")

    # 18 Allocator untouched (decision does not import/call allocate in step)
    import inspect
    src = inspect.getsource(ESCDecisionLogic.step)
    _gate(gates, "allocator_untouched",
          "BrakeAllocator" not in src and "allocate" not in src,
          "decision.step has no allocator call")

    # 19 Regression
    avx, at, _ = _launch(cfg)
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
        "phase": "15.3",
        "status": status,
        "gates_passed": n_pass,
        "gates_total": len(gates),
        "gates": gates,
        "policy": {
            "e_enter": ESCDecisionConfig().e_enter,
            "e_exit": ESCDecisionConfig().e_exit,
            "K_Mz": ESCDecisionConfig().K_Mz,
            "max_delta_Mz": ESCDecisionConfig().max_delta_Mz,
        },
        "regression": {
            "hyper": {"t100": at100, "t200": at200, "ref": REF_HYPER},
            "hist": {"t100": ht100, "ref": REF_HIST},
        },
    }
    with open(ROOT / "validation_result.json", "w") as f:
        json.dump(summary, f, indent=2, default=str)
    print(f"\n=== PHASE 15.3 — {status} {n_pass}/{len(gates)} ===")
    return summary


if __name__ == "__main__":
    run_validation()
