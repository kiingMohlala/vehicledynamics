"""
Phase 15.6 — ESC controller characterization harness.

No plant modification. No gain freeze. Architecture untouched.
"""
from __future__ import annotations

from dataclasses import dataclass, asdict
from typing import Any

import numpy as np

from vehicle_dynamics.controls.esc_closed_loop import ClosedLoopESC, ClosedLoopESCConfig
from vehicle_dynamics.controls.esc_command import ESCCommand, BrakeAllocator
from vehicle_dynamics.controls.esc_observability import ESCObservability


@dataclass
class CharResult:
    K_Mz: float
    vx0: float
    steer: float
    Mz_dist: float
    mu_mode: str
    e0: float
    e_final: float
    e_peak: float
    e_reduction: float
    settle_s: float | None
    max_Mz: float
    brake_energy: float
    sat_fraction: float
    util_peak: float
    mz_flips: int
    max_cmd: float

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def characterize_run(
    sim_factory,
    *,
    K_Mz: float = 4000.0,
    vx0: float = 25.0,
    steer: float = 0.06,
    Mz_dist: float = -3000.0,
    mu_per_wheel=None,
    settle_n: int = 40,
    dist_n: int = 25,
    recover_n: int = 180,
    thr: float = 0.12,
) -> CharResult:
    """
    Mild steer → yaw disturbance → ESC recovery.
    sim_factory() must return a fresh Simulation.
    """
    alloc = BrakeAllocator()
    sim = sim_factory()
    sim.mu_per_wheel = mu_per_wheel
    sim.reset(vx0, 3)

    for _ in range(settle_n):
        sim.esc_brake_add = None
        sim._step_plant(thr, 0, steer, 1, 0, 0.01)
    for _ in range(dist_n):
        sim.esc_brake_add = alloc.allocate(ESCCommand(Mz_dist)).brake_cmd
        sim._step_plant(thr, 0, steer, 1, 0, 0.01)

    e0 = abs(ESCObservability().observe_from_simulation(sim).e_r)
    esc = ClosedLoopESC(ClosedLoopESCConfig(enabled=True, K_Mz=float(K_Mz)))
    e_hist, mz_hist, util_hist, cmd_hist = [], [], [], []
    brake_energy = 0.0
    sat_steps = 0
    for _ in range(recover_n):
        esc.step(sim)
        add = sim.esc_brake_add if sim.esc_brake_add is not None else np.zeros(4)
        mx = float(np.max(add))
        cmd_hist.append(mx)
        if mx >= 0.99:
            sat_steps += 1
        mz_hist.append(esc.last_Mz)
        o = esc.observer.last
        e_hist.append(abs(o.e_r))
        util_hist.append(o.util_max)
        brake_energy += float(np.sum(add)) * 0.01
        sim._step_plant(thr, 0, steer, 1, 0, 0.01)

    e_arr = np.asarray(e_hist, dtype=float)
    settle = None
    for i in range(len(e_arr) - 20):
        if np.all(e_arr[i:i + 20] < 0.06):
            settle = i * 0.01
            break
    flips = 0
    for i in range(1, len(mz_hist)):
        if (
            mz_hist[i] * mz_hist[i - 1] < 0
            and abs(mz_hist[i]) > 50
            and abs(mz_hist[i - 1]) > 50
        ):
            flips += 1

    mu_mode = "split" if mu_per_wheel is not None else "nominal"
    return CharResult(
        K_Mz=float(K_Mz),
        vx0=float(vx0),
        steer=float(steer),
        Mz_dist=float(Mz_dist),
        mu_mode=mu_mode,
        e0=float(e0),
        e_final=float(e_arr[-1]) if len(e_arr) else float(e0),
        e_peak=float(np.max(e_arr[:40])) if len(e_arr) else float(e0),
        e_reduction=float(e0 - e_arr[-1]) if len(e_arr) else 0.0,
        settle_s=settle,
        max_Mz=float(np.max(np.abs(mz_hist))) if mz_hist else 0.0,
        brake_energy=float(brake_energy),
        sat_fraction=float(sat_steps / max(len(cmd_hist), 1)),
        util_peak=float(np.max(util_hist)) if util_hist else 0.0,
        mz_flips=int(flips),
        max_cmd=float(np.max(cmd_hist)) if cmd_hist else 0.0,
    )


def kmz_sweep(sim_factory, gains: list[float] | None = None, **kwargs) -> list[CharResult]:
    gains = gains or [1000.0, 2000.0, 4000.0, 8000.0, 12000.0]
    return [characterize_run(sim_factory, K_Mz=k, **kwargs) for k in gains]


def recommend_candidate(results: list[CharResult], sat_cap: float = 0.5) -> CharResult:
    pool = [r for r in results if r.sat_fraction < sat_cap] or list(results)
    return min(pool, key=lambda r: r.e_final)
