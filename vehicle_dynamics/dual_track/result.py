from dataclasses import dataclass
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
    # Per-wheel arrays shape (n, 4) order FL, FR, RL, RR
    kappa: np.ndarray
    alpha: np.ndarray
    Fx: np.ndarray
    Fy: np.ndarray
    Fz: np.ndarray
    omega: np.ndarray
    utilization: np.ndarray
    # Phase 5.2 brake diagnostics
    brake_torque: np.ndarray = None   # (n, 4) commanded torque after ABS
    abs_pressure: np.ndarray = None   # (n, 4) ABS pressure factors
    X: np.ndarray = None
    Y: np.ndarray = None
