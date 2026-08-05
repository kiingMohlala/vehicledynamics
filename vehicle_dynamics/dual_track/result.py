"""Dual-track simulation result container."""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np


@dataclass
class DualTrackResult:
    time: np.ndarray
    vx: np.ndarray
    vy: np.ndarray
    r: np.ndarray
    psi: np.ndarray
    delta: np.ndarray
    delta_fl: np.ndarray
    delta_fr: np.ndarray
    pedal: np.ndarray
    kappa: np.ndarray
    alpha: np.ndarray
    Fx: np.ndarray
    Fy: np.ndarray
    Fz: np.ndarray
    omega: np.ndarray
    utilization: np.ndarray
    brake_torque: np.ndarray
    abs_pressure: np.ndarray
    X: np.ndarray
    Y: np.ndarray
    # Phase 7.1 – per-wheel camber used by tire [rad]
    camber: np.ndarray = field(default_factory=lambda: np.zeros((0, 4)))
