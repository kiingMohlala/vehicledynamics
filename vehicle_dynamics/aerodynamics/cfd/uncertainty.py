"""Interpolation confidence / uncertainty estimates."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class UncertaintyEstimate:
    confidence: float = 0.0       # 0..1
    interp_distance: float = np.inf
    data_density: float = 0.0     # samples (normalized)
    sigma_Cd: float = 0.0         # rough uncertainty

    @property
    def high_confidence(self) -> bool:
        return self.confidence >= 0.7


def estimate_uncertainty(
    distance: float,
    n_samples: int,
    in_bounds: bool,
    *,
    ref_distance: float = 1.0,
) -> UncertaintyEstimate:
    if n_samples <= 0 or not np.isfinite(distance):
        return UncertaintyEstimate(confidence=0.0, interp_distance=distance)

    # Closer → higher confidence; density soft bonus
    dist_score = float(np.exp(-distance / max(ref_distance, 1e-6)))
    dens_score = float(np.clip(n_samples / 30.0, 0.0, 1.0))
    conf = 0.7 * dist_score + 0.3 * dens_score
    if not in_bounds:
        conf *= 0.4
    conf = float(np.clip(conf, 0.0, 1.0))
    sigma = 0.02 * (1.0 - conf) + 0.005
    return UncertaintyEstimate(
        confidence=conf,
        interp_distance=float(distance),
        data_density=dens_score,
        sigma_Cd=sigma,
    )
