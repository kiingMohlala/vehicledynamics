"""Racing line generation from track centerline."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .track import Track
from .curvature import curvature_from_xy, reference_speed


@dataclass
class RacingLine:
    s: np.ndarray
    x: np.ndarray
    y: np.ndarray
    curvature: np.ndarray
    v_ref: np.ndarray
    offset: np.ndarray          # lateral offset from centerline (m)
    kind: str = "center"        # center | ideal | imported | user

    @property
    def length(self) -> float:
        return float(self.s[-1]) if len(self.s) else 0.0


def center_line(track: Track, mu: float = 1.0, v_max: float = 80.0) -> RacingLine:
    kappa = track.curvature
    return RacingLine(
        s=track.s.copy(), x=track.x.copy(), y=track.y.copy(),
        curvature=kappa, v_ref=reference_speed(kappa, mu=mu, v_max=v_max),
        offset=np.zeros_like(track.s), kind="center",
    )


def ideal_line(track: Track, mu: float = 1.0, v_max: float = 80.0, aggressiveness: float = 0.35) -> RacingLine:
    """
    Simple ideal-line heuristic: bias toward outside on entry, inside at apex, outside on exit.
    offset = -aggressiveness * sign(κ) * width/2 * smooth window
    """
    kappa = track.curvature
    w = track.width
    # Smooth curvature sign for apex estimate
    k_s = np.convolve(kappa, np.ones(5) / 5.0, mode="same")
    offset = -aggressiveness * np.sign(k_s) * 0.5 * w
    # Blend near straights
    offset = np.where(np.abs(k_s) < 0.01, 0.0, offset)
    n = track.heading + np.pi / 2
    x = track.x + offset * np.cos(n)
    y = track.y + offset * np.sin(n)
    k2 = curvature_from_xy(x, y)
    return RacingLine(
        s=track.s.copy(), x=x, y=y, curvature=k2,
        v_ref=reference_speed(k2, mu=mu, v_max=v_max),
        offset=offset, kind="ideal",
    )


def from_xy(s: np.ndarray, x: np.ndarray, y: np.ndarray, mu: float = 1.0, v_max: float = 80.0, kind: str = "imported") -> RacingLine:
    k = curvature_from_xy(x, y)
    return RacingLine(
        s=np.asarray(s, dtype=float), x=np.asarray(x, dtype=float), y=np.asarray(y, dtype=float),
        curvature=k, v_ref=reference_speed(k, mu=mu, v_max=v_max),
        offset=np.zeros_like(s, dtype=float), kind=kind,
    )
