"""
Phase 14.9.1 — Steering Authority & Ackermann Geometry.
No retuning of 14.8 frozen vehicle identity.
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
from vehicle_dynamics.steering.steering_config import SteeringConfig
from vehicle_dynamics.steering.steering_model import SteeringModel

ROOT = Path("artifacts/phase_14_9_1")
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


def run_validation() -> dict:
    ROOT.mkdir(parents=True, exist_ok=True)
    gates = []
    hyper = bind_authoritative_hypercar(1100.0, 750.0)
    hist = bind_historical_demonstrator()
    cfg = hyper.simulation_config
    sim = Simulation(cfg)
    sim.reset(0.0, 1)
    st_model = sim.dual_track.steering

    # 01 config authority
    sc = st_model.cfg
    _gate(gates, "steering_config_authority",
          abs(sc.max_steer_angle - cfg.max_steer_angle) < 1e-9
          and abs(sc.steering_rate - cfg.steering_rate) < 1e-9
          and sc.ackermann_enabled == cfg.ackermann_enabled,
          f"max={sc.max_steer_angle} rate={sc.steering_rate} ack={sc.ackermann_enabled}")

    # 02 runtime parameter authority
    _gate(gates, "runtime_parameter_authority",
          abs(sc.wheelbase - (sim.dual_track.cfg.a + sim.dual_track.cfg.b)) < 1e-6
          and abs(sc.track_front - sim.dual_track.cfg.track_f) < 1e-6,
          f"L={sc.wheelbase} Tf={sc.track_front}")

    # 03 max angle limit
    m = SteeringModel(SteeringConfig(max_steer_angle=0.30, steering_rate=10.0))
    for _ in range(20):
        m.step(1.0, 0.1)
    _gate(gates, "max_angle_limit", abs(m.state.actual - 0.30) < 1e-9,
          f"actual={m.state.actual}")

    # 04 steering rate limit
    m = SteeringModel(SteeringConfig(max_steer_angle=1.0, steering_rate=0.5))
    angles = []
    for _ in range(10):
        m.step(1.0, 0.1)
        angles.append(m.state.actual)
    rate_ok = all(abs(angles[i] - (angles[i - 1] if i else 0)) <= 0.05 + 1e-9 for i in range(len(angles)))
    _gate(gates, "steering_rate_limit", rate_ok and angles[-1] < 0.6,
          f"angles={angles[:4]}...{angles[-1]:.3f}")

    # 05 zero command
    m = SteeringModel()
    m.step(0.2, 1.0)
    m.step(0.0, 10.0)
    _gate(gates, "zero_command", abs(m.state.actual) < 1e-9 and abs(m.state.delta_fl) < 1e-9,
          f"actual={m.state.actual}")

    # 06 positive command
    m = SteeringModel(SteeringConfig(steering_rate=5.0))
    m.step(0.15, 1.0)
    _gate(gates, "positive_command", m.state.actual > 0 and m.state.delta_fl > 0,
          f"actual={m.state.actual:.4f}")

    # 07 negative command
    m = SteeringModel(SteeringConfig(steering_rate=5.0))
    m.step(-0.15, 1.0)
    _gate(gates, "negative_command", m.state.actual < 0 and m.state.delta_fr < 0,
          f"actual={m.state.actual:.4f}")

    # 08 left/right symmetry
    m1 = SteeringModel(SteeringConfig(steering_rate=5.0, ackermann_enabled=True))
    m2 = SteeringModel(SteeringConfig(steering_rate=5.0, ackermann_enabled=True))
    m1.step(0.2, 1.0)
    m2.step(-0.2, 1.0)
    sym = (
        abs(m1.state.delta_fl + m2.state.delta_fr) < 1e-9
        and abs(m1.state.delta_fr + m2.state.delta_fl) < 1e-9
    )
    _gate(gates, "left_right_symmetry", sym,
          f"L: FL={m1.state.delta_fl:.5f} FR={m1.state.delta_fr:.5f}")

    # 09 ackermann enabled
    m = SteeringModel(SteeringConfig(ackermann_enabled=True, steering_rate=5.0))
    m.step(0.25, 1.0)
    _gate(gates, "ackermann_enabled",
          abs(m.state.delta_fl - m.state.delta_fr) > 1e-4,
          f"FL={m.state.delta_fl:.5f} FR={m.state.delta_fr:.5f}")

    # 10 inner/outer geometry (left turn: |FL| > |FR|)
    _gate(gates, "ackermann_inner_outer_geometry",
          abs(m.state.delta_fl) > abs(m.state.delta_fr),
          f"|FL|={abs(m.state.delta_fl):.5f} |FR|={abs(m.state.delta_fr):.5f}")

    # 11 ackermann disabled
    m = SteeringModel(SteeringConfig(ackermann_enabled=False, steering_rate=5.0))
    m.step(0.25, 1.0)
    _gate(gates, "ackermann_disabled",
          abs(m.state.delta_fl - m.state.delta_fr) < 1e-12,
          f"FL={m.state.delta_fl:.5f} FR={m.state.delta_fr:.5f}")

    # 12 wheel angle runtime authority (plant)
    s = Simulation(cfg)
    s.reset(20.0, 3)
    for _ in range(50):
        s._step_plant(0.1, 0, 0.25, 1, 0, 0.01)
    d = s.dual_track.diagnostics()
    _gate(gates, "wheel_angle_runtime_authority",
          abs(d["delta_fl"]) > 0.1 and abs(d["delta_fl"] - d["delta_fr"]) > 1e-4,
          f"FL={d['delta_fl']:.4f} FR={d['delta_fr']:.4f}")

    # 13 poisoned default
    DualTrackConfig.__dataclass_fields__["max_steer_angle"].default = 0.01
    DualTrackConfig.__dataclass_fields__["steering_rate"].default = 0.01
    try:
        s = Simulation(bind_authoritative_hypercar().simulation_config)
        ok = (
            abs(s.dual_track.steering.cfg.max_steer_angle - 0.52) < 1e-6
            and abs(s.dual_track.steering.cfg.steering_rate - 1.2) < 1e-6
        )
        _gate(gates, "poisoned_default_resistance", ok,
              f"max={s.dual_track.steering.cfg.max_steer_angle} rate={s.dual_track.steering.cfg.steering_rate}")
    finally:
        DualTrackConfig.__dataclass_fields__["max_steer_angle"].default = 0.52
        DualTrackConfig.__dataclass_fields__["steering_rate"].default = 1.2

    # Mutation: max ×0.5
    c = bind_authoritative_hypercar().simulation_config
    c.max_steer_angle = 0.26
    s = Simulation(c)
    s.reset(20, 3)
    for _ in range(80):
        s._step_plant(0, 0, 1.0, 1, 0, 0.01)
    _gate(gates, "mutation_max_angle",
          abs(s.dual_track.steering.state.actual - 0.26) < 1e-6,
          f"actual={s.dual_track.steering.state.actual}")

    # Mutation: rate ×0.5 → slower
    def settle_steps(rate):
        c = bind_authoritative_hypercar().simulation_config
        c.steering_rate = rate
        c.max_steer_angle = 0.4
        s = Simulation(c)
        s.reset(20, 3)
        for i in range(200):
            s._step_plant(0, 0, 0.4, 1, 0, 0.01)
            if abs(s.dual_track.steering.state.actual - 0.4) < 1e-4:
                return i
        return 200
    n_fast = settle_steps(2.0)
    n_slow = settle_steps(0.5)
    _gate(gates, "mutation_steering_rate", n_slow > n_fast,
          f"steps rate2={n_fast} rate0.5={n_slow}")

    # 14 determinism
    runs = []
    for _ in range(5):
        s = Simulation(cfg)
        s.reset(20.0, 3)
        for __ in range(40):
            s._step_plant(0.1, 0, 0.2, 1, 0, 0.01)
        st = s.dual_track.steering.state
        runs.append((round(st.actual, 9), round(st.delta_fl, 9), round(st.delta_fr, 9)))
    _gate(gates, "deterministic_replay", len(set(runs)) == 1, f"run0={runs[0]}")

    # 15 historical isolation
    hvx, ht, _ = _launch(hist.simulation_config)
    ht100, ht200 = _t_to(hvx, ht, 27.78), _t_to(hvx, ht, 55.56)
    _gate(gates, "historical_isolation",
          ht100 is not None and abs(ht100 - REF_HIST[0]) < 0.3,
          f"t100={ht100} t200={ht200}")

    # 16 zero-steer 14.8 regression
    avx, at, _ = _launch(cfg)
    at100, at200 = _t_to(avx, at, 27.78), _t_to(avx, at, 55.56)
    reg_ok = (
        at100 is not None and abs(at100 - REF_HYPER[0]) < 0.15
        and at200 is not None and abs(at200 - REF_HYPER[1]) < 0.25
    )
    _gate(gates, "zero_steer_14_8_regression", reg_ok,
          f"t100={at100} t200={at200} ref={REF_HYPER}")

    # 17 no vehicle identity mutation
    id_ok = (
        abs(cfg.mass - 1100) < 1e-6
        and abs(cfg.peak_power_kw - 750) < 1e-6
        and abs(cfg.mu_tire - 1.15) < 1e-6
    )
    _gate(gates, "no_vehicle_identity_mutation", id_ok,
          f"m={cfg.mass} P={cfg.peak_power_kw} μ={cfg.mu_tire}")

    n_pass = sum(1 for g in gates if g["pass"])
    status = "PASS" if n_pass == len(gates) else (
        "PASS WITH LIMITATIONS" if n_pass >= len(gates) - 1 else "FAIL"
    )
    summary = {
        "phase": "14.9.1",
        "status": status,
        "gates_passed": n_pass,
        "gates_total": len(gates),
        "gates": gates,
        "regression": {
            "hyper": {"t100": at100, "t200": at200, "ref": REF_HYPER},
            "hist": {"t100": ht100, "t200": ht200, "ref": REF_HIST},
        },
    }
    with open(ROOT / "validation_result.json", "w") as f:
        json.dump(summary, f, indent=2, default=str)
    print(f"\n=== PHASE 14.9.1 — {status} {n_pass}/{len(gates)} ===")
    return summary


if __name__ == "__main__":
    run_validation()
