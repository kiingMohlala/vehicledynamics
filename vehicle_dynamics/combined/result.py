from dataclasses import dataclass
import numpy as np

@dataclass
class CombinedSimulationResult:
    time: np.ndarray
    vx: np.ndarray
    vy: np.ndarray
    r: np.ndarray
    psi: np.ndarray
    delta: np.ndarray
    pedal: np.ndarray
    alpha_f: np.ndarray
    alpha_r: np.ndarray
    kappa_f: np.ndarray
    kappa_r: np.ndarray
    Fx_f: np.ndarray
    Fx_r: np.ndarray
    Fy_f: np.ndarray
    Fy_r: np.ndarray
    ay_force: np.ndarray
    X: np.ndarray
    Y: np.ndarray
    stopping_distance: float = 0.0
