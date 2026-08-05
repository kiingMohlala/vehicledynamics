"""Occupant-cell intrusion metrics from nodal displacements."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from vehicle_dynamics.fem.assembler import Model


@dataclass
class IntrusionMetrics:
    seat_disp_m: float = 0.0
    harness_disp_m: float = 0.0
    survival_cell_intrusion_m: float = 0.0
    steering_column_disp_m: float = 0.0
    pedal_intrusion_m: float = 0.0
    max_node_disp_m: float = 0.0
    peak_decel_g: float = 0.0


def _node_disp(model: Model, u: np.ndarray, tag: str) -> float:
    try:
        n = model.get_node(tag)
    except Exception:
        return 0.0
    b = 6 * n.id
    return float(np.linalg.norm(u[b : b + 3]))


def compute_intrusion(
    model: Model,
    u: np.ndarray,
    *,
    speed_mps: float = 0.0,
    crush_distance: float = 0.0,
) -> IntrusionMetrics:
    max_d = 0.0
    for n in model.nodes:
        b = 6 * n.id
        max_d = max(max_d, float(np.linalg.norm(u[b : b + 3])))

    seat = max(
        _node_disp(model, u, "seat_front_left"),
        _node_disp(model, u, "seat_rear_left"),
        _node_disp(model, u, "seat_front_right"),
    )
    harness = max(
        _node_disp(model, u, "harness_left"),
        _node_disp(model, u, "harness_right"),
    )
    # Survival cell: average roof node motion
    roof = 0.0
    n_roof = 0
    for tag in (
        "front_roof_left",
        "front_roof_right",
        "rear_roof_left",
        "rear_roof_right",
    ):
        d = _node_disp(model, u, tag)
        if d > 0:
            roof += d
            n_roof += 1
    roof = roof / max(n_roof, 1)

    dash = _node_disp(model, u, "dash_left")
    pedal = _node_disp(model, u, "front_lower_left")

    # Peak decel estimate: v²/(2 s) if crush distance known
    peak_g = 0.0
    if crush_distance > 1e-6 and speed_mps > 0:
        peak_g = (speed_mps**2) / (2.0 * crush_distance) / 9.81

    return IntrusionMetrics(
        seat_disp_m=seat,
        harness_disp_m=harness,
        survival_cell_intrusion_m=roof,
        steering_column_disp_m=dash,
        pedal_intrusion_m=pedal,
        max_node_disp_m=max_d,
        peak_decel_g=peak_g,
    )
