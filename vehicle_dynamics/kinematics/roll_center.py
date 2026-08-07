"""Roll center from instant centers and contact patches."""
from __future__ import annotations

import numpy as np

from .instant_center import front_view_ic


def roll_center_height(
    ic_yz: np.ndarray,
    contact_yz: np.ndarray,
    track_half: float | None = None,
) -> float:
    """
    Classic geometric RC: line from IC to contact patch intersects vehicle centerline (y=0).
    Returns z height of roll center.
    """
    ic = np.asarray(ic_yz, dtype=float)
    cp = np.asarray(contact_yz, dtype=float)
    dy = cp[0] - ic[0]
    if abs(dy) < 1e-12:
        return float(cp[1])
    # parametric: y(t) = ic_y + t*(cp_y - ic_y) = 0
    t = -ic[0] / dy
    z = ic[1] + t * (cp[1] - ic[1])
    return float(z)


def roll_axis(rc_front_z: float, rc_rear_z: float, wheelbase: float) -> dict:
    """Roll axis inclination (rad) from front/rear RC heights."""
    if abs(wheelbase) < 1e-9:
        incl = 0.0
    else:
        incl = float(np.arctan2(rc_rear_z - rc_front_z, wheelbase))
    return {
        "rc_front": rc_front_z,
        "rc_rear": rc_rear_z,
        "inclination_rad": incl,
        "inclination_deg": float(np.degrees(incl)),
    }
