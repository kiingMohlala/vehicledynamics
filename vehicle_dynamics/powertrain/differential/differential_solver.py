"""
Differential coordinator: transmission wheel torque → left/right wheel torques.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np

from .differential_types import DiffType
from .open_diff import open_split
from .locked_diff import locked_split
from .clutch_lsd import clutch_lsd_split
from .viscous_diff import viscous_split
from .torsen import torsen_split
from .torque_vectoring import torque_vector_split
from .axle_model import AxleModel
from .wheel_speed import axle_speed, differential_speed
from .differential import DiffResult


@dataclass
class DifferentialConfig:
    enabled: bool = True
    diff_type: str = "open"
    preload: float = 80.0
    torque_bias_ratio: float = 3.0
    k_lock: float = 15.0
    k_viscous: float = 8.0
    max_bias: float = 400.0
    coast_lock: float = 0.6
    max_tv_delta: float = 500.0
    radius: float = 0.32


@dataclass
class DrivelineState:
    """Standardized driveline output for ESC / TC / AWD."""

    torque_left: float = 0.0
    torque_right: float = 0.0
    torque_input: float = 0.0
    axle_speed: float = 0.0
    delta_omega: float = 0.0
    bias: float = 0.0
    locking_fraction: float = 0.0
    diff_type: str = "open"
    efficiency: float = 1.0
    yaw_moment_proxy: float = 0.0  # (T_R - T_L) / track diagnostic


@dataclass
class DifferentialState:
    result: DiffResult = field(default_factory=DiffResult)
    driveline: DrivelineState = field(default_factory=DrivelineState)


class DifferentialSolver:
    def __init__(self, config: DifferentialConfig | None = None):
        self.cfg = config or DifferentialConfig()
        self.axle = AxleModel()
        self.state = DifferentialState()
        self._tv_delta = 0.0

    def set_tv_delta(self, delta_T: float) -> None:
        """External TV / ESC yaw bias command (N·m)."""
        self._tv_delta = float(delta_T)

    def step(
        self,
        input_torque: float,
        omega_left: float,
        omega_right: float,
        dt: float = 0.01,
        *,
        mu_left: float = 1.0,
        mu_right: float = 1.0,
        Fz_left: float = 4000.0,
        Fz_right: float = 4000.0,
        delta_T: float | None = None,
    ) -> DifferentialState:
        cfg = self.cfg
        if not cfg.enabled:
            self.state = DifferentialState()
            return self.state

        T_in = float(input_torque)
        wL, wR = float(omega_left), float(omega_right)
        dtype = DiffType(cfg.diff_type.lower().replace("-", "_").replace(" ", "_"))

        bias = 0.0
        if dtype == DiffType.OPEN:
            T_L, T_R = open_split(T_in)
        elif dtype == DiffType.LOCKED:
            T_L, T_R = locked_split(
                T_in, mu_left, mu_right, Fz_left, Fz_right, cfg.radius
            )
        elif dtype == DiffType.CLUTCH_LSD:
            T_L, T_R, bias = clutch_lsd_split(
                T_in, wL, wR,
                preload=cfg.preload, k_lock=cfg.k_lock, max_bias=cfg.max_bias,
            )
        elif dtype == DiffType.VISCOUS:
            T_L, T_R, bias = viscous_split(T_in, wL, wR, k_v=cfg.k_viscous)
        elif dtype == DiffType.TORSEN:
            T_L, T_R, bias = torsen_split(
                T_in, wL, wR,
                tbr=cfg.torque_bias_ratio, preload=cfg.preload * 0.5,
                coast_lock=cfg.coast_lock,
            )
        elif dtype == DiffType.TORQUE_VECTORING:
            d = self._tv_delta if delta_T is None else delta_T
            T_L, T_R, bias = torque_vector_split(
                T_in, d, max_delta=cfg.max_tv_delta
            )
        else:
            T_L, T_R = open_split(T_in)

        # Traction limit clamp (soft)
        cap_L = mu_left * Fz_left * cfg.radius
        cap_R = mu_right * Fz_right * cfg.radius
        T_L_c = float(np.clip(T_L, -cap_L, cap_L))
        T_R_c = float(np.clip(T_R, -cap_R, cap_R))

        w_axle = axle_speed(wL, wR)
        dw = differential_speed(wL, wR)
        self.axle.step(T_in, T_L_c, T_R_c, wL, wR, dt)

        # Locking fraction: 0 open-like, 1 fully locked speeds
        lock_frac = 0.0
        if dtype == DiffType.LOCKED:
            lock_frac = 1.0
        elif abs(T_in) > 1e-6:
            lock_frac = float(np.clip(abs(T_L_c - T_R_c) / (abs(T_in) + 1e-9), 0.0, 1.0))

        res = DiffResult(
            torque_left=T_L_c,
            torque_right=T_R_c,
            bias=float(bias),
            locking_fraction=lock_frac,
            axle_speed=w_axle,
            delta_omega=dw,
        )
        track = 1.55
        driveline = DrivelineState(
            torque_left=T_L_c,
            torque_right=T_R_c,
            torque_input=T_in,
            axle_speed=w_axle,
            delta_omega=dw,
            bias=float(bias),
            locking_fraction=lock_frac,
            diff_type=dtype.value,
            efficiency=1.0 if abs(T_in) < 1e-9 else float(
                np.clip((T_L_c + T_R_c) / T_in, 0.0, 1.05)
            ),
            yaw_moment_proxy=(T_R_c - T_L_c) / track,
        )
        self.state = DifferentialState(result=res, driveline=driveline)
        return self.state
