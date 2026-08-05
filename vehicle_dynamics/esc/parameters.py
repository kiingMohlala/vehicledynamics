"""Phase 5.3 – ESC parameters."""

from dataclasses import dataclass


@dataclass
class ESCParameters:
    # Reference model
    understeer_grad: float = 0.0025   # K_us [rad·s²/m²] for r_ss = vx*δ/(L(1+K_us*vx²))
    r_ref_tau: float = 0.10          # first-order lag on r_ref [s]
    r_ref_max: float = 0.80          # |r_ref| clamp [rad/s]

    # Activation
    yaw_deadband: float = 0.04       # |e_r| below this → inactive [rad/s]
    min_speed: float = 5.0           # ESC off below this [m/s]
    beta_limit: float = 0.15         # |β| soft limit [rad] (sideslip assist)

    # Controller
    Kp_yaw: float = 8000.0           # N·m / (rad/s)
    Kd_yaw: float = 400.0            # N·m / (rad/s²) approx via discrete derivative
    Mz_max: float = 6000.0           # |Mz| saturation [N·m]

    # Allocator
    front_os_share: float = 0.70     # fraction of Mz on outer front for oversteer
    rear_us_share: float = 0.70      # fraction of Mz on inner rear for understeer
    max_brake_scale: float = 0.85    # max additional brake fraction per wheel

    # Hysteresis
    on_threshold: float = 0.05       # |e_r| to activate
    off_threshold: float = 0.025     # |e_r| to deactivate (must be < on)
