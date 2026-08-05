"""Helical / Torsen approximation with torque bias ratio."""

from __future__ import annotations

import numpy as np

from .open_diff import open_split


def torsen_split(
    T_in: float,
    omega_L: float,
    omega_R: float,
    *,
    tbr: float = 3.0,
    preload: float = 40.0,
    coast_lock: float = 0.6,
) -> tuple[float, float, float]:
    """
    Torque Bias Ratio: max(T_hi/T_lo) ≤ TBR.
    Preload provides static bias; coast_lock scales when T_in < 0.
    """
    T_L0, T_R0 = open_split(T_in)
    dw = omega_L - omega_R
    lock = 1.0 if T_in >= 0 else float(np.clip(coast_lock, 0.0, 1.0))
    tbr_eff = max(tbr * lock, 1.0)

    if abs(dw) < 1e-6 and abs(T_in) < 1e-6:
        return T_L0, T_R0, 0.0

    # Desired: more torque to slower wheel, limited by TBR
    # T_slow / T_fast ≤ TBR is automatic if we set fractions
    # Split: slow gets tbr/(1+tbr), fast gets 1/(1+tbr) of |T_in|
    if abs(T_in) < 1e-9:
        # Preload only couples speeds weakly
        bias = preload * np.sign(dw) if abs(dw) > 1e-9 else 0.0
        return T_L0 - 0.5 * bias, T_R0 + 0.5 * bias, abs(bias)

    slow_frac = tbr_eff / (1.0 + tbr_eff)
    fast_frac = 1.0 / (1.0 + tbr_eff)
    if dw > 0:
        # L faster → R is slow
        T_L = T_in * fast_frac
        T_R = T_in * slow_frac
    elif dw < 0:
        T_L = T_in * slow_frac
        T_R = T_in * fast_frac
    else:
        T_L, T_R = T_L0, T_R0

    # Add preload coupling
    if abs(dw) > 1e-9:
        bias = preload * lock * np.sign(dw)
        T_L -= 0.5 * bias
        T_R += 0.5 * bias
    else:
        bias = 0.0

    return float(T_L), float(T_R), float(abs(bias) + abs(T_L - T_R) * 0.5)
