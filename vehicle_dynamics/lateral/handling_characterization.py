"""
Phase 14.9.8 — Passive understeer/oversteer characterization.

Steering gradient: dδ/d(ay)
  > 0  understeer
  ≈ 0  neutral
  < 0  oversteer

No controllers. No retuning.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np


@dataclass
class HandlingPoint:
    delta: float
    vx: float
    ay: float
    r: float
    alpha_front: float
    alpha_rear: float
    Fy_front: float
    Fy_rear: float
    util_front: float
    util_rear: float
    yaw_gain: float  # r / delta (if delta ≠ 0)


@dataclass
class HandlingSweep:
    points: list[HandlingPoint] = field(default_factory=list)
    steering_gradient: float = 0.0  # dδ/d(ay)  [rad / (m/s²)]
    classification: str = "unknown"
    linear_region: list[HandlingPoint] = field(default_factory=list)
    limit_region: list[HandlingPoint] = field(default_factory=list)


def classify_gradient(grad: float, neutral_tol: float = 0.002) -> str:
    if grad > neutral_tol:
        return "understeer"
    if grad < -neutral_tol:
        return "oversteer"
    return "neutral"


def compute_steering_gradient(points: list[HandlingPoint]) -> float:
    """
    Fit δ = g · ay + b over points with sufficient |ay|.
    Returns g = dδ/d(ay).
    """
    if len(points) < 2:
        return 0.0
    ays = np.array([p.ay for p in points])
    deltas = np.array([p.delta for p in points])
    # Prefer positive ay branch for sign consistency
    mask = ays > 0.5
    if mask.sum() < 2:
        mask = np.abs(ays) > 0.5
    if mask.sum() < 2:
        return 0.0
    a, d = ays[mask], deltas[mask]
    # linear fit
    A = np.vstack([a, np.ones_like(a)]).T
    g, _ = np.linalg.lstsq(A, d, rcond=None)[0]
    return float(g)


def run_constant_speed_steer_sweep(
    sim_factory,
    *,
    vx: float,
    deltas: list[float],
    settle_n: int = 160,
    meas_n: int = 40,
    throttle_base: float = 0.12,
) -> HandlingSweep:
    """
    For each commanded centreline δ, settle at approximately constant speed
    and record steady-state handling metrics.
    """
    points: list[HandlingPoint] = []
    for delta in deltas:
        sim = sim_factory()
        sim.reset(vx, 3)
        # pre-settle straight
        for _ in range(30):
            err = vx - sim.state.vehicle.vx
            thr = float(np.clip(throttle_base + 0.05 * err, 0.0, 0.6))
            sim._step_plant(thr, 0.0, 0.0, 1.0, 0.0, 0.01)
        for _ in range(settle_n):
            err = vx - sim.state.vehicle.vx
            thr = float(np.clip(throttle_base + 0.05 * err, 0.0, 0.6))
            sim._step_plant(thr, 0.0, float(delta), 1.0, 0.0, 0.01)
        # measure
        ays, rs, alf, alr, fyf, fyr, utf, utr, acts = [], [], [], [], [], [], [], [], []
        for _ in range(meas_n):
            err = vx - sim.state.vehicle.vx
            thr = float(np.clip(throttle_base + 0.05 * err, 0.0, 0.6))
            sim._step_plant(thr, 0.0, float(delta), 1.0, 0.0, 0.01)
            d = sim.dual_track.diagnostics()
            v = sim.state.vehicle
            ays.append(v.ay)
            rs.append(v.yaw_rate)
            alf.append(0.5 * (d["alpha_FL"] + d["alpha_FR"]))
            alr.append(0.5 * (d["alpha_RL"] + d["alpha_RR"]))
            fyf.append(d["Fy_front"])
            fyr.append(d["Fy_rear"])
            utf.append(0.5 * (d["utilization"][0] + d["utilization"][1]))
            utr.append(0.5 * (d["utilization"][2] + d["utilization"][3]))
            acts.append(d["steer_actual"])
        ay = float(np.mean(ays))
        r = float(np.mean(rs))
        act = float(np.mean(acts))
        yg = r / act if abs(act) > 1e-6 else 0.0
        points.append(HandlingPoint(
            delta=act,
            vx=float(np.mean([sim.state.vehicle.vx])),
            ay=ay,
            r=r,
            alpha_front=float(np.mean(alf)),
            alpha_rear=float(np.mean(alr)),
            Fy_front=float(np.mean(fyf)),
            Fy_rear=float(np.mean(fyr)),
            util_front=float(np.mean(utf)),
            util_rear=float(np.mean(utr)),
            yaw_gain=yg,
        ))

    # Split linear vs limit by utilization
    linear, limit = [], []
    for p in points:
        if max(p.util_front, p.util_rear) < 0.85:
            linear.append(p)
        else:
            limit.append(p)
    use = linear if len(linear) >= 2 else points
    grad = compute_steering_gradient(use)
    return HandlingSweep(
        points=points,
        steering_gradient=grad,
        classification=classify_gradient(grad),
        linear_region=linear,
        limit_region=limit,
    )


def yaw_gain_vs_speed(
    sim_factory,
    *,
    delta: float = 0.06,
    speeds: list[float] | None = None,
) -> list[dict[str, float]]:
    speeds = speeds or [15.0, 20.0, 25.0, 30.0]
    out = []
    for vx in speeds:
        sw = run_constant_speed_steer_sweep(
            sim_factory, vx=vx, deltas=[delta], settle_n=140, meas_n=30,
        )
        p = sw.points[0]
        out.append({
            "vx": vx,
            "ay": p.ay,
            "r": p.r,
            "yaw_gain": p.yaw_gain,
            "util_front": p.util_front,
            "util_rear": p.util_rear,
        })
    return out
