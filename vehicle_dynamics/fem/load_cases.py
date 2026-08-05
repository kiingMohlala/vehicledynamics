"""Engineering load-case generators for tube-frame analysis."""

from __future__ import annotations

import numpy as np
from .assembler import Model
from .constraints import fix_node, apply_force
from .solver import solve_static
from .result import StaticResult


def _zeros(model: Model) -> np.ndarray:
    return np.zeros(model.ndof)


def torsional_rig(
    model: Model,
    fixed_tags: tuple[str, str] = ("susp_rl", "susp_rr"),
    load_tags: tuple[str, str] = ("susp_fl", "susp_fr"),
    couple_force_N: float = 1000.0,
) -> tuple[StaticResult, dict]:
    """
    Chassis torsional-rig style test:
      fix rear pickups, ±Fz couple at front pickups.
    Returns result + metrics including stiffness Nm/deg.
    """
    for tag in fixed_tags:
        fix_node(model.get_node(tag))

    left = model.get_node(load_tags[0])
    right = model.get_node(load_tags[1])
    track = abs(right.y - left.y)

    F = _zeros(model)
    apply_force(F, left, fz=couple_force_N)
    apply_force(F, right, fz=-couple_force_N)

    res = solve_static(model, F)
    if not res.success:
        return res, {"error": res.message}

    z_l = res.node_displacement(left.id)[2]
    z_r = res.node_displacement(right.id)[2]
    twist_rad = float(np.arctan2(z_l - z_r, track))
    twist_deg = float(np.degrees(twist_rad))
    torque_Nm = couple_force_N * track
    if abs(twist_deg) < 1e-12:
        stiff = float("inf")
    else:
        stiff = torque_Nm / twist_deg

    res.torsional_stiffness_Nm_per_deg = stiff
    metrics = {
        "twist_deg": twist_deg,
        "applied_torque_Nm": torque_Nm,
        "stiffness_Nm_per_deg": stiff,
        "track_m": track,
    }
    return res, metrics


def cornering(
    model: Model,
    lateral_g: float = 1.2,
    vehicle_mass_kg: float = 1400.0,
    fixed_tags: tuple[str, ...] = ("susp_rl", "susp_rr", "susp_fl", "susp_fr"),
) -> tuple[StaticResult, dict]:
    """
    Approximate steady-state cornering: lateral loads at suspension pickups
    proportional to static weight share, then release front for flexibility
    study — here we fix all four and apply Fy at body-side nodes via susp tags.
    """
    # Fix rear only, load all four laterally (simplified)
    for tag in ("susp_rl", "susp_rr"):
        fix_node(model.get_node(tag))

    total_Fy = vehicle_mass_kg * 9.81 * lateral_g
    F = _zeros(model)
    # 60/40 front bias on lateral load for illustration
    per_front = 0.30 * total_Fy
    per_rear = 0.20 * total_Fy
    apply_force(F, model.get_node("susp_fl"), fy=per_front)
    apply_force(F, model.get_node("susp_fr"), fy=per_front)
    apply_force(F, model.get_node("susp_rl"), fy=per_rear)
    apply_force(F, model.get_node("susp_rr"), fy=per_rear)

    res = solve_static(model, F)
    return res, {"total_Fy": total_Fy, "lateral_g": lateral_g}


def braking(
    model: Model,
    decel_g: float = 1.0,
    vehicle_mass_kg: float = 1400.0,
) -> tuple[StaticResult, dict]:
    """Longitudinal loads into front pickups; rear fixed."""
    for tag in ("susp_rl", "susp_rr"):
        fix_node(model.get_node(tag))

    total_Fx = -vehicle_mass_kg * 9.81 * decel_g
    F = _zeros(model)
    apply_force(F, model.get_node("susp_fl"), fx=0.55 * total_Fx)
    apply_force(F, model.get_node("susp_fr"), fx=0.45 * total_Fx)
    # Optional engine inertia share into cradle
    try:
        eng = model.get_node("engine_left")
        apply_force(F, eng, fx=0.1 * total_Fx)
    except KeyError:
        pass

    res = solve_static(model, F)
    return res, {"total_Fx": total_Fx, "decel_g": decel_g}


def acceleration(
    model: Model,
    accel_g: float = 0.6,
    vehicle_mass_kg: float = 1400.0,
) -> tuple[StaticResult, dict]:
    for tag in ("susp_fl", "susp_fr"):
        fix_node(model.get_node(tag))

    total_Fx = vehicle_mass_kg * 9.81 * accel_g
    F = _zeros(model)
    apply_force(F, model.get_node("susp_rl"), fx=0.5 * total_Fx)
    apply_force(F, model.get_node("susp_rr"), fx=0.5 * total_Fx)
    res = solve_static(model, F)
    return res, {"total_Fx": total_Fx, "accel_g": accel_g}


def vertical_landing(
    model: Model,
    impact_g: float = 3.0,
    vehicle_mass_kg: float = 1400.0,
) -> tuple[StaticResult, dict]:
    """Four-wheel bump: fix nothing structurally via pins at all susp, apply -Fz."""
    for tag in ("susp_fl", "susp_fr", "susp_rl", "susp_rr"):
        # Pin translations only so moments can develop in tubes
        n = model.get_node(tag)
        n.fixed[0] = True
        n.fixed[1] = True
        n.fixed[2] = True

    Fz_each = -vehicle_mass_kg * 9.81 * impact_g / 4.0
    F = _zeros(model)
    # Apply equal share through nearby structure nodes (loads already on fixed
    # nodes become reactions; apply on harness/seat as body inertia proxy)
    try:
        apply_force(F, model.get_node("harness_left"), fz=Fz_each)
        apply_force(F, model.get_node("harness_right"), fz=Fz_each)
        apply_force(F, model.get_node("seat_front_left"), fz=Fz_each)
        apply_force(F, model.get_node("seat_front_right"), fz=Fz_each)
    except KeyError:
        for tag in ("susp_fl", "susp_fr", "susp_rl", "susp_rr"):
            apply_force(F, model.get_node(tag), fz=Fz_each)

    res = solve_static(model, F)
    return res, {"impact_g": impact_g, "Fz_each": Fz_each}


def harness_load(
    model: Model,
    force_N: float = 7000.0,
) -> tuple[StaticResult, dict]:
    """
    FIA-style shoulder-harness load into harness bar (forward + down component).
    Rear lower mounts fixed.
    """
    for tag in ("rear_lower_left", "rear_lower_right"):
        try:
            fix_node(model.get_node(tag))
        except KeyError:
            pass
    for tag in ("susp_rl", "susp_rr"):
        try:
            fix_node(model.get_node(tag))
        except KeyError:
            pass

    F = _zeros(model)
    # Split between left/right harness attachments
    fx = -0.7 * force_N / 2.0
    fz = -0.3 * force_N / 2.0
    apply_force(F, model.get_node("harness_left"), fx=fx, fz=fz)
    apply_force(F, model.get_node("harness_right"), fx=fx, fz=fz)

    res = solve_static(model, F)
    return res, {"force_N": force_N}
