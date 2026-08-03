"""Phase 5.4 – Torque vectoring parameters."""

from dataclasses import dataclass


@dataclass
class TVParameters:
    # Powertrain
    max_total_drive_torque: float = 4000.0   # N·m at wheels (sum)
    front_drive_fraction: float = 0.0        # 0 = RWD, 0.5 = AWD, 1 = FWD

    # Modes: "open" | "fixed_bias" | "active_rear"
    mode: str = "active_rear"

    # Fixed bias (positive = more torque to left)
    fixed_left_fraction: float = 0.5         # of axle torque to left wheel

    # Active rear TV
    Kp_yaw: float = 2000.0                   # N·m / (rad/s) yaw-error gain
    Kd_yaw: float = 100.0
    max_delta_T: float = 1200.0              # max |T_left - T_right| on rear [N·m]
    yaw_deadband: float = 0.03               # rad/s
    min_speed: float = 3.0                   # m/s
    min_throttle: float = 0.05

    # Reference understeer (match ESC convention)
    understeer_grad: float = 0.0025
    r_ref_max: float = 0.80
