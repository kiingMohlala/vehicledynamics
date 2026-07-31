from dataclasses import dataclass
import numpy as np

@dataclass
class LateralSimulationResult:
    time: np.ndarray
    vx: np.ndarray
    vy: np.ndarray
    r: np.ndarray
    psi: np.ndarray
    delta: np.ndarray
    alpha_f: np.ndarray
    alpha_r: np.ndarray
    Fy_f: np.ndarray
    Fy_r: np.ndarray
    ay: np.ndarray                 # lateral acceleration at CG
    X: np.ndarray                  # inertial path
    Y: np.ndarray
