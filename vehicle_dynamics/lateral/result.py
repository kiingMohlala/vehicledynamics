from dataclasses import dataclass
import numpy as np

@dataclass
class LateralSimulationResult:
    """Time histories from a bicycle-model simulation."""
    time: np.ndarray                 # [s]
    vx: np.ndarray                   # longitudinal speed [m/s] (held constant)
    vy: np.ndarray                   # lateral velocity at CG [m/s]
    r: np.ndarray                    # yaw rate [rad/s]
    psi: np.ndarray                  # yaw angle [rad]
    delta: np.ndarray                # steering angle [rad]
    alpha_f: np.ndarray              # front slip angle [rad]
    alpha_r: np.ndarray              # rear slip angle [rad]
    Fy_f: np.ndarray                 # front lateral force [N]
    Fy_r: np.ndarray                 # rear lateral force [N]
    ay_force: np.ndarray             # (Fy_f + Fy_r) / m  [m/s²]
    ay_vehicle: np.ndarray           # vy_dot + vx * r   [m/s²]
    X: np.ndarray                    # inertial X [m]
    Y: np.ndarray                    # inertial Y [m]
    # Phase 4.1 load-transfer diagnostics (logging only; do not affect dynamics)
    dFz_front: np.ndarray = None     # front transfer magnitude [N]
    dFz_rear: np.ndarray = None      # rear transfer magnitude [N]
    Fz_fl: np.ndarray = None         # front-left normal load [N]
    Fz_fr: np.ndarray = None         # front-right normal load [N]
    Fz_rl: np.ndarray = None         # rear-left normal load [N]
    Fz_rr: np.ndarray = None         # rear-right normal load [N]
    wheel_lift_front: np.ndarray = None  # bool flags
    wheel_lift_rear: np.ndarray = None
