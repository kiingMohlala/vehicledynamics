"""
Core handling metric helpers (Phase 7.2).

Analysis only — does not modify vehicle physics.
"""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

G = 9.81


@dataclass
class SteadyStateMetrics:
    understeer_gradient_deg_per_g: float  # K [deg/g]
    yaw_rate_gain: float                  # r / ay [rad/s per m/s²] or /g noted in report
    steering_gain: float                  # ay / delta [m/s² per rad]
    max_ay: float                         # [m/s²]
    max_ay_g: float
    turning_radius: float                 # [m]
    characteristic_speed: float | None    # [m/s] if understeer
    critical_speed: float | None          # [m/s] if oversteer
    yaw_rate_ss: float
    ay_ss: float
    delta_ss: float
    vx_ss: float


@dataclass
class UtilizationMetrics:
    peak: np.ndarray          # (4,)
    mean: np.ndarray
    time_above_90: np.ndarray  # fraction of samples
    limiting_wheel: int       # 0..3
    limiting_axle: str        # "front" | "rear" | "balanced"
    front_mean: float
    rear_mean: float


@dataclass
class StabilityMetrics:
    peak_yaw_rate: float
    rms_yaw_rate: float
    peak_beta_deg: float
    rms_beta_deg: float
    peak_load_transfer: float | None = None
    peak_jacking: float | None = None
    peak_rc_migration: float | None = None


@dataclass
class DriverMetrics:
    max_steer_rad: float
    max_steer_deg: float
    entry_speed: float
    exit_speed: float
    average_speed: float
    corner_time: float
    stopping_distance: float | None = None
    stop_100_0_kmh: float | None = None


def sideslip_beta(vx: np.ndarray, vy: np.ndarray) -> np.ndarray:
    return np.arctan2(vy, np.maximum(np.abs(vx), 0.5))


def rms(x: np.ndarray) -> float:
    x = np.asarray(x, dtype=float)
    if x.size == 0:
        return 0.0
    return float(np.sqrt(np.mean(x**2)))


def understeer_gradient(
    delta: float,
    L: float,
    R: float,
    ay: float,
) -> float:
    """
    K ≈ (δ - L/R) / (ay/g)   [rad/g] then reported in deg/g.

    Neutral: K=0; understeer K>0; oversteer K<0.
    """
    if abs(ay) < 0.5 or abs(R) < 1.0:
        return 0.0
    ackermann = L / R
    K_rad_per_g = (float(delta) - ackermann) / (float(ay) / G)
    return float(np.degrees(K_rad_per_g))
