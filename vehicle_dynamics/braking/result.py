from dataclasses import dataclass
import numpy as np

@dataclass
class BrakeSimulationResult:
    time: np.ndarray
    vehicle_speed: np.ndarray
    wheel_speed_front: np.ndarray
    wheel_speed_rear: np.ndarray
    slip_front: np.ndarray
    slip_rear: np.ndarray
    pressure_front: np.ndarray
    pressure_rear: np.ndarray
    brake_torque_front: np.ndarray
    brake_torque_rear: np.ndarray
    tire_force_front: np.ndarray
    tire_force_rear: np.ndarray
    deceleration: np.ndarray
    stopping_distance: float
    peak_slip_front: float
    peak_slip_rear: float
    tire_model: str = "unknown"
