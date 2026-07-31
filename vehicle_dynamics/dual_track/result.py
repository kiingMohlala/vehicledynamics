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
    pedal: np.ndarray
    # Per-wheel arrays shape (n, 4) order FL, FR, RL, RR
    kappa: np.ndarray
    alpha: np.ndarray
    Fx: np.ndarray
    Fy: np.ndarray
    Fz: np.ndarray
    omega: np.ndarray
    utilization: np.ndarray
    X: np.ndarray
    Y: np.ndarray
