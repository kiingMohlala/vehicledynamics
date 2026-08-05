"""Crash load-case generators (quasi-static force patterns)."""

from __future__ import annotations

import numpy as np
from vehicle_dynamics.fem.assembler import Model
from vehicle_dynamics.fem.constraints import fix_node, apply_force


def _zeros(model: Model) -> np.ndarray:
    return np.zeros(model.ndof)


def _fix_tags(model: Model, tags: tuple[str, ...]) -> None:
    for t in tags:
        try:
            fix_node(model.get_node(t))
        except KeyError:
            pass


def frontal_impact(model: Model, force_N: float = 50e3) -> np.ndarray:
    """Rear fixed; +Fx on front lower / susp nodes (vehicle → barrier reaction)."""
    _fix_tags(model, ("susp_rl", "susp_rr", "rear_lower_left", "rear_lower_right"))
    F = _zeros(model)
    for tag in ("susp_fl", "susp_fr", "front_lower_left", "front_lower_right"):
        try:
            apply_force(F, model.get_node(tag), fx=-force_N / 4.0)
        except KeyError:
            continue
    return F


def rear_impact(model: Model, force_N: float = 40e3) -> np.ndarray:
    _fix_tags(model, ("susp_fl", "susp_fr", "front_lower_left", "front_lower_right"))
    F = _zeros(model)
    for tag in ("susp_rl", "susp_rr", "rear_lower_left", "rear_lower_right"):
        try:
            apply_force(F, model.get_node(tag), fx=force_N / 4.0)
        except KeyError:
            continue
    return F


def side_impact(model: Model, force_N: float = 45e3, side: str = "left") -> np.ndarray:
    if side == "left":
        supports = ("susp_fr", "susp_rr", "front_lower_right", "rear_lower_right")
        loads = ("susp_fl", "front_lower_left", "rear_lower_left", "door_left")
        fy = force_N
    else:
        supports = ("susp_fl", "susp_rl", "front_lower_left", "rear_lower_left")
        loads = ("susp_fr", "front_lower_right", "rear_lower_right", "door_right")
        fy = -force_N
    _fix_tags(model, supports)
    F = _zeros(model)
    active = 0
    for tag in loads:
        try:
            model.get_node(tag)
            active += 1
        except KeyError:
            pass
    active = max(active, 1)
    for tag in loads:
        try:
            apply_force(F, model.get_node(tag), fy=fy / active)
        except KeyError:
            continue
    return F


def roof_crush(model: Model, force_N: float = 30e3) -> np.ndarray:
    _fix_tags(model, ("susp_fl", "susp_fr", "susp_rl", "susp_rr"))
    F = _zeros(model)
    for tag in (
        "front_roof_left",
        "front_roof_right",
        "rear_roof_left",
        "rear_roof_right",
    ):
        try:
            apply_force(F, model.get_node(tag), fz=-force_N / 4.0)
        except KeyError:
            continue
    return F


def harness_pull(model: Model, force_N: float = 7e3) -> np.ndarray:
    _fix_tags(model, ("susp_rl", "susp_rr", "rear_lower_left", "rear_lower_right"))
    F = _zeros(model)
    for tag in ("harness_left", "harness_right"):
        try:
            apply_force(F, model.get_node(tag), fx=-0.7 * force_N / 2, fz=-0.3 * force_N / 2)
        except KeyError:
            continue
    return F
