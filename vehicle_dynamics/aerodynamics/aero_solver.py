"""High-level aero solver and vehicle coupling helpers."""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np

from .coefficients import AeroConfig
from .ride_height import RideHeightState
from .aero_model import AeroState, compute_aero_loads
from .aero_map import AeroMap, build_default_map


@dataclass
class AeroResult:
    state: AeroState
    speed: float
    ride: RideHeightState
    config: AeroConfig
    # Axle load deltas for vehicle coupling (positive = extra downward load on tires)
    dFz_front: float = 0.0
    dFz_rear: float = 0.0
    drag_force: float = 0.0
    side_force: float = 0.0
    drag_power: float = 0.0  # W

    @property
    def enabled(self) -> bool:
        return self.config.enabled


def solve_aero(
    speed: float,
    cfg: AeroConfig | None = None,
    ride: RideHeightState | None = None,
    aero_map: AeroMap | None = None,
) -> AeroResult:
    """
    Compute aero loads and coupling deltas for the vehicle model.

    Tire normal-load coupling:
      dFz_front = -state.Fz_front  (extra downward force on front tires)
      dFz_rear  = -state.Fz_rear
    """
    cfg = cfg or AeroConfig()
    ride = ride or RideHeightState(
        h_front=cfg.h_front_ref, h_rear=cfg.h_rear_ref
    )

    coeffs = None
    if aero_map is not None and cfg.enabled and speed > 0:
        coeffs = aero_map.evaluate(speed, ride.h_front, ride.h_rear, ride.yaw_rad)

    st = compute_aero_loads(speed, cfg, ride=ride, coeffs=coeffs)

    return AeroResult(
        state=st,
        speed=speed,
        ride=ride,
        config=cfg,
        dFz_front=-st.Fz_front,
        dFz_rear=-st.Fz_rear,
        drag_force=st.drag,
        side_force=st.Fy,
        drag_power=st.drag * max(speed, 0.0),
    )


def axle_load_deltas(result: AeroResult) -> tuple[float, float]:
    """(dFz_front, dFz_rear) extra downward tire load [N]."""
    return result.dFz_front, result.dFz_rear
