"""Suspension mounts and upright placeholders from hardpoints / parametric layout."""
from __future__ import annotations

from typing import Any
import numpy as np
from .component import Component


def suspension_mounts(
    wheelbase: float = 2.70,
    track: float = 1.55,
    ride_height: float = 0.12,
) -> list[Component]:
    """Create simple suspension upright/mount components at four corners."""
    half_t = track * 0.5
    corners = {
        "FL": np.array([0.0, half_t, ride_height]),
        "FR": np.array([0.0, -half_t, ride_height]),
        "RL": np.array([-wheelbase, half_t, ride_height]),
        "RR": np.array([-wheelbase, -half_t, ride_height]),
    }
    # shift so front axle at x=0 was used; place front near x small positive for packaging
    # use front axle at 0, rear at -wheelbase relative — for assembly we put front at ~0.05*wb
    parts = []
    axles = {
        "FL": np.array([0.05 * wheelbase, half_t, ride_height]),
        "FR": np.array([0.05 * wheelbase, -half_t, ride_height]),
        "RL": np.array([0.05 * wheelbase - wheelbase, half_t, ride_height]),
        "RR": np.array([0.05 * wheelbase - wheelbase, -half_t, ride_height]),
    }
    for name, pos in axles.items():
        parts.append(Component(
            name=f"upright_{name}",
            category="suspension",
            position=pos,
            size=np.array([0.15, 0.12, 0.25]),
            mass=8.0,
            meta={"corner": name},
        ))
        parts.append(Component(
            name=f"control_arm_{name}",
            category="suspension",
            position=pos + np.array([0.0, -0.08 * np.sign(pos[1] + 1e-9), 0.0]),
            size=np.array([0.35, 0.08, 0.05]),
            mass=3.5,
            meta={"corner": name},
        ))
    return parts
