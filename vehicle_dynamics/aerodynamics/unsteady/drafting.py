"""Slipstream / drafting factors vs following distance."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class DraftingParams:
    drag_reduction_max: float = 0.35      # max fraction of drag removed
    downforce_loss_max: float = 0.40      # max fraction of DF lost
    cooling_loss_max: float = 0.50
    wake_length: float = 25.0             # m characteristic recovery
    min_gap: float = 2.0                  # m


def drafting_factors(
    gap_m: float,
    params: DraftingParams | None = None,
) -> dict[str, float]:
    """
    Returns multipliers:
      Cd_factor, Cl_factor, cooling_factor  in (0, 1]
    Closer gap → lower Cd and lower |Cl|.
    """
    p = params or DraftingParams()
    if gap_m <= 0 or not np.isfinite(gap_m):
        return {"Cd_factor": 1.0, "Cl_factor": 1.0, "cooling_factor": 1.0}

    # Exponential recovery with distance
    x = max(gap_m - p.min_gap, 0.0) / max(p.wake_length, 1e-6)
    strength = float(np.exp(-x))  # 1 at bumper, →0 far away

    Cd_f = 1.0 - p.drag_reduction_max * strength
    Cl_f = 1.0 - p.downforce_loss_max * strength
    cool = 1.0 - p.cooling_loss_max * strength
    return {
        "Cd_factor": float(np.clip(Cd_f, 0.4, 1.0)),
        "Cl_factor": float(np.clip(Cl_f, 0.3, 1.0)),
        "cooling_factor": float(np.clip(cool, 0.2, 1.0)),
        "wake_strength": strength,
    }
