"""
Phase 16.1 — Baseline handling scenario suite harness.

Runs identical scenarios ESC OFF vs ESC ON (K_Mz candidate).
No plant modification.
"""
from __future__ import annotations

from dataclasses import dataclass, field, asdict
from typing import Any, Callable

import numpy as np

from vehicle_dynamics.controls.esc_closed_loop import ClosedLoopESC, ClosedLoopESCConfig


@dataclass
class ScenarioMetrics:
    name: str
    esc_on: bool
    vx0: float
    peak_er: float = 0.0
    final_er: float = 0.0
    peak_beta: float = 0.0
    peak_r: float = 0.0
    peak_ay: float = 0.0
    max_Mz: float = 0.0
    mz_flips: int = 0
    max_cmd: float = 0.0
    util_peak: float = 0.0
    active_steps: int = 0
    finite: bool = True
    min_Fz: float = 0.0

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _esc(enabled: bool, K_Mz: float = 10000.0) -> ClosedLoopESC:
    return ClosedLoopESC(ClosedLoopESCConfig(enabled=enabled, K_Mz=K_Mz))


def _run_loop(
    sim_factory: Callable,
    *,
    enabled: bool,
    vx0: float,
    n: int,
    input_fn: Callable[[int, Any], tuple[float, float, float]],
    mu_per_wheel=None,
    K_Mz: float = 10000.0,
    name: str = "",
) -> ScenarioMetrics:
    sim = sim_factory()
    sim.mu_per_wheel = mu_per_wheel
    sim.reset(vx0, 3)
    esc = _esc(enabled, K_Mz)
    peak_er = peak_beta = peak_r = peak_ay = 0.0
    max_Mz = max_cmd = util_peak = 0.0
    active_steps = 0
    mz_prev = 0.0
    flips = 0
    finite = True
    last_er = 0.0

    for i in range(n):
        thr, brk, steer = input_fn(i, sim)
        esc.step(sim)
        sim._step_plant(thr, brk, steer, 1.0, 0.0, 0.01)
        o = esc.observer.last if enabled else None
        v = sim.state.vehicle
        if enabled and o is not None:
            er = abs(o.e_r)
            beta = abs(o.beta)
            last_er = er
            peak_er = max(peak_er, er)
            peak_beta = max(peak_beta, beta)
            util_peak = max(util_peak, o.util_max)
            max_Mz = max(max_Mz, abs(esc.last_Mz))
            if esc.last_active:
                active_steps += 1
            if esc.last_Mz * mz_prev < 0 and abs(esc.last_Mz) > 50 and abs(mz_prev) > 50:
                flips += 1
            mz_prev = esc.last_Mz
            if sim.esc_brake_add is not None:
                max_cmd = max(max_cmd, float(np.max(sim.esc_brake_add)))
        else:
            # OFF: still track kinematics
            peak_beta = max(peak_beta, abs(np.arctan2(v.vy, max(abs(v.vx), 0.5))))
            last_er = 0.0
        peak_r = max(peak_r, abs(v.yaw_rate))
        peak_ay = max(peak_ay, abs(v.ay))
        if not np.isfinite(v.yaw_rate) or not np.isfinite(v.ay):
            finite = False
            break

    d = sim.dual_track.diagnostics()
    return ScenarioMetrics(
        name=name,
        esc_on=enabled,
        vx0=vx0,
        peak_er=peak_er,
        final_er=last_er,
        peak_beta=peak_beta,
        peak_r=peak_r,
        peak_ay=peak_ay,
        max_Mz=max_Mz,
        mz_flips=flips,
        max_cmd=max_cmd,
        util_peak=util_peak,
        active_steps=active_steps,
        finite=finite,
        min_Fz=float(d.get("min_Fz", 0.0)),
    )


def step_steer(sim_factory, vx0: float, delta: float, enabled: bool, **kw) -> ScenarioMetrics:
    def inp(i, sim):
        st = 0.0 if i < 30 else delta
        err = vx0 - sim.state.vehicle.vx
        thr = float(np.clip(0.12 + 0.05 * err, 0, 0.6))
        return thr, 0.0, st
    return _run_loop(sim_factory, enabled=enabled, vx0=vx0, n=200, input_fn=inp,
                     name=f"step_steer_vx{vx0}_d{delta}_{'ON' if enabled else 'OFF'}", **kw)


def sine_steer(sim_factory, vx0: float, amp: float, freq: float, enabled: bool, **kw) -> ScenarioMetrics:
    def inp(i, sim):
        st = amp * np.sin(2 * np.pi * freq * i * 0.01)
        err = vx0 - sim.state.vehicle.vx
        thr = float(np.clip(0.12 + 0.05 * err, 0, 0.6))
        return thr, 0.0, float(st)
    return _run_loop(sim_factory, enabled=enabled, vx0=vx0, n=250, input_fn=inp,
                     name=f"sine_vx{vx0}_A{amp}_{'ON' if enabled else 'OFF'}", **kw)


def lane_change(sim_factory, vx0: float, amp: float, enabled: bool, **kw) -> ScenarioMetrics:
    # simple double lane-change profile
    def inp(i, sim):
        t = i * 0.01
        if t < 0.5:
            st = 0.0
        elif t < 1.2:
            st = amp
        elif t < 1.8:
            st = -amp
        elif t < 2.5:
            st = amp * 0.5
        else:
            st = 0.0
        err = vx0 - sim.state.vehicle.vx
        thr = float(np.clip(0.12 + 0.05 * err, 0, 0.6))
        return thr, 0.0, float(st)
    return _run_loop(sim_factory, enabled=enabled, vx0=vx0, n=300, input_fn=inp,
                     name=f"lane_change_vx{vx0}_{'ON' if enabled else 'OFF'}", **kw)


def steady_corner(sim_factory, vx0: float, delta: float, enabled: bool, **kw) -> ScenarioMetrics:
    def inp(i, sim):
        st = 0.0 if i < 40 else delta
        err = vx0 - sim.state.vehicle.vx
        thr = float(np.clip(0.12 + 0.05 * err, 0, 0.6))
        return thr, 0.0, st
    return _run_loop(sim_factory, enabled=enabled, vx0=vx0, n=250, input_fn=inp,
                     name=f"corner_vx{vx0}_d{delta}_{'ON' if enabled else 'OFF'}", **kw)


def straight_brake(sim_factory, vx0: float, brk: float, enabled: bool, **kw) -> ScenarioMetrics:
    def inp(i, sim):
        return 0.0, brk if i > 20 else 0.0, 0.0
    return _run_loop(sim_factory, enabled=enabled, vx0=vx0, n=150, input_fn=inp,
                     name=f"brake_vx{vx0}_{'ON' if enabled else 'OFF'}", **kw)


def brake_steer(sim_factory, vx0: float, brk: float, delta: float, enabled: bool, **kw) -> ScenarioMetrics:
    def inp(i, sim):
        st = 0.0 if i < 30 else delta
        b = 0.0 if i < 40 else brk
        return 0.0, b, st
    return _run_loop(sim_factory, enabled=enabled, vx0=vx0, n=180, input_fn=inp,
                     name=f"brake_steer_vx{vx0}_{'ON' if enabled else 'OFF'}", **kw)


def recovery_vs_free(sim_factory, vx0: float, Mz_dist: float, enabled: bool, **kw) -> ScenarioMetrics:
    from vehicle_dynamics.controls.esc_command import ESCCommand, BrakeAllocator
    alloc = BrakeAllocator()

    def inp(i, sim):
        # pre: settle, then disturb 25 steps, then recover
        if 40 <= i < 65:
            sim.esc_brake_add = alloc.allocate(ESCCommand(Mz_dist)).brake_cmd
        err = vx0 - sim.state.vehicle.vx
        thr = float(np.clip(0.12 + 0.05 * err, 0, 0.6))
        return thr, 0.0, 0.06

    return _run_loop(sim_factory, enabled=enabled, vx0=vx0, n=220, input_fn=inp,
                     name=f"recovery_vx{vx0}_{'ON' if enabled else 'OFF'}", **kw)
