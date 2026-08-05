"""
Load-sensitive friction (Phase 7.5).

μ_eff(Fz) = μ0 · (Fz0 / Fz)^n

At Fz = Fz0 → μ_eff = μ0
Higher load → slightly lower friction coefficient
Lower load → slightly higher friction coefficient
"""

from __future__ import annotations

import numpy as np


def effective_mu(
    mu0: float,
    Fz: float,
    Fz0: float = 4000.0,
    exponent: float = 0.08,
    mu_min: float = 0.05,
    mu_max: float = 2.5,
) -> float:
    """
    Load-sensitive friction coefficient.

    Parameters
    ----------
    mu0 : nominal friction at reference load
    Fz : current normal load [N]
    Fz0 : reference load [N]
    exponent : load sensitivity exponent n (typical 0.05–0.15)
    mu_min, mu_max : hard clamps for numerical safety
    """
    Fz = max(float(Fz), 1.0)
    Fz0 = max(float(Fz0), 1.0)
    n = float(exponent)
    mu0 = float(mu0)

    if abs(n) < 1e-12:
        mu = mu0
    else:
        mu = mu0 * (Fz0 / Fz) ** n

    if not np.isfinite(mu):
        mu = mu0
    return float(np.clip(mu, mu_min, mu_max))
