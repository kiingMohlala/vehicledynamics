"""
Phase 6.1 – Motion ratio and effective wheel rate / damping.

Definitions (vertical, positive bump = wheel up):

  MR(z) = dz_wheel / dz_spring

For a linear spring/damper mounted such that spring travel is related to
wheel travel by a constant geometric ratio r = z_spring / z_wheel:

  MR = 1 / r                    (if r = spring/wheel)

Energy equivalence gives the standard results:

  Kw = Ks * (dz_s / dz_w)^2 = Ks * r^2 = Ks / MR^2

Careful with convention:
  Many texts define MR = spring_travel / wheel_travel  (installation ratio),
  others define MR = wheel_travel / spring_travel.

This module uses:

  installation_ratio  IR = z_spring / z_wheel
  motion_ratio        MR = z_wheel / z_spring = 1 / IR

  Kw = Ks * IR^2 = Ks / MR^2
  Cw = Cs * IR^2 = Cs / MR^2

Direct-acting (coilover on upright): IR = 1, MR = 1, Kw = Ks, Cw = Cs.
Pushrod with IR = 0.7 (spring moves less than wheel): Kw = 0.49 * Ks.
"""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class SpringDamperParams:
    Ks: float = 30000.0   # spring rate [N/m]
    Cs: float = 2000.0    # damping rate [N·s/m]


@dataclass
class MotionRatioParams:
    """
    Linear layout parameters.

    installation_ratio: z_spring / z_wheel at the operating point.
      1.0 = direct-acting
      <1  = pushrod/rocker (spring moves less than wheel)
      >1  = rare over-driven layouts
    """
    installation_ratio: float = 1.0
    layout: str = "direct"   # "direct" | "pushrod" | "custom"


@dataclass
class WheelRateResult:
    installation_ratio: float
    motion_ratio: float
    Ks: float
    Cs: float
    Kw: float
    Cw: float
    layout: str

    def summary(self) -> str:
        return (
            f"layout={self.layout}  IR={self.installation_ratio:.4f}  "
            f"MR={self.motion_ratio:.4f}  "
            f"Ks={self.Ks:.1f} → Kw={self.Kw:.1f} N/m  "
            f"Cs={self.Cs:.1f} → Cw={self.Cw:.1f} N·s/m"
        )


def motion_ratio_from_ir(installation_ratio: float) -> float:
    """MR = z_wheel / z_spring = 1 / IR."""
    ir = float(installation_ratio)
    if abs(ir) < 1e-12:
        raise ValueError("installation_ratio must be non-zero")
    return 1.0 / ir


def installation_ratio_from_travels(
    wheel_travel: float,
    spring_travel: float,
) -> float:
    """IR = z_spring / z_wheel from measured travels (same sign convention)."""
    zw = float(wheel_travel)
    if abs(zw) < 1e-12:
        raise ValueError("wheel_travel must be non-zero")
    return float(spring_travel) / zw


def effective_wheel_rate(Ks: float, installation_ratio: float) -> float:
    """Kw = Ks * IR²."""
    ir = float(installation_ratio)
    return float(Ks) * ir * ir


def effective_wheel_damping(Cs: float, installation_ratio: float) -> float:
    """Cw = Cs * IR²."""
    ir = float(installation_ratio)
    return float(Cs) * ir * ir


def compute_wheel_rate(
    spring: SpringDamperParams = None,
    mr_params: MotionRatioParams = None,
) -> WheelRateResult:
    """
    Compute effective wheel spring and damper rates for a linear layout.
    """
    spring = spring or SpringDamperParams()
    mr_params = mr_params or MotionRatioParams()
    ir = float(mr_params.installation_ratio)
    if abs(ir) < 1e-12:
        raise ValueError("installation_ratio must be non-zero")
    mr = 1.0 / ir
    Kw = effective_wheel_rate(spring.Ks, ir)
    Cw = effective_wheel_damping(spring.Cs, ir)
    return WheelRateResult(
        installation_ratio=ir,
        motion_ratio=mr,
        Ks=float(spring.Ks),
        Cs=float(spring.Cs),
        Kw=Kw,
        Cw=Cw,
        layout=mr_params.layout,
    )


def wheel_rate_curve(
    Ks: float,
    installation_ratios: np.ndarray,
) -> np.ndarray:
    """Kw(IR) for a sweep of installation ratios (e.g. vs travel later)."""
    ir = np.asarray(installation_ratios, dtype=float)
    return Ks * ir ** 2
