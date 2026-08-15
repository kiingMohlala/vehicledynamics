"""
Phase 14.9.5 — Mechanical anti-roll bar.

Equal/opposite vertical forces across an axle; no net vertical load.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


class AntiRollBar(Protocol):
    def forces(self, z_left: float, z_right: float,
               z_dot_left: float, z_dot_right: float) -> tuple[float, float, float]:
        """Return (F_left, F_right, M_roll) on the body (up +)."""
        ...


@dataclass
class MechanicalAntiRollBar:
    """
    Torsional ARB abstraction.

    Δz = z_left − z_right  (body corner height from static, +up)
    F_pair = K·Δz + C·Δż
    F_left  = +F_pair   (up on body left when left is higher → resists)
    F_right = −F_pair
    M_roll  = F_pair * track   (opposes roll when consistent with sign)
    """

    k_arb: float = 25000.0   # N/m equivalent vertical stiffness pair
    c_arb: float = 500.0     # N·s/m
    track: float = 1.65
    enabled: bool = True

    def forces(
        self,
        z_left: float,
        z_right: float,
        z_dot_left: float = 0.0,
        z_dot_right: float = 0.0,
    ) -> tuple[float, float, float]:
        if not self.enabled or (abs(self.k_arb) < 1e-9 and abs(self.c_arb) < 1e-9):
            return 0.0, 0.0, 0.0
        dz = z_left - z_right
        dz_dot = z_dot_left - z_dot_right
        # Force on left body attachment (up +): resists positive left rise
        f_pair = self.k_arb * dz + self.c_arb * dz_dot
        f_left = -f_pair   # left higher → push left down
        f_right = +f_pair  # right lower → push right up
        # Roll moment on body (+φ = right side down): opposing moment
        # When φ>0, right down, z_right < z_left, dz>0, f_pair>0
        # f_left=-ve, f_right=+ve → left down, right up → reduces φ
        M = f_right * (self.track / 2) - f_left * (self.track / 2)
        return f_left, f_right, M




@dataclass
class HydraulicAntiRollBar:
    """
    Passive hydraulic cross-linked ARB (no pump / no active pressure).

    Model:
      Relative piston displacement δ = z_left − z_right
      Effective hydraulic stiffness K_h from fluid compressibility:
          K_h = (2 · β · A²) / V   (lumped both chambers)
      Orifice damping C_h from restriction:
          F_damp = C_h · δ̇

      F_left  = −(K_h · δ + C_h · δ̇)
      F_right = +(K_h · δ + C_h · δ̇)
      ΣF = 0 always (load transfer only)

    Parameters can be set directly (k_hyd, c_hyd) or derived from
    A (m²), V (m³), beta (Pa), orifice coefficient.
    """

    k_hyd: float = 30000.0      # N/m effective
    c_hyd: float = 800.0        # N·s/m
    track: float = 1.65
    # Optional physical parameters (if k_hyd <= 0, derived)
    piston_area: float = 0.0    # m²
    chamber_volume: float = 0.0 # m³
    bulk_modulus: float = 1.5e9 # Pa (oil ~1–1.8 GPa)
    enabled: bool = True
    # Dissipation accumulator (J)
    E_dissipated: float = 0.0

    def effective_stiffness(self) -> float:
        if self.k_hyd > 0:
            return self.k_hyd
        if self.piston_area > 0 and self.chamber_volume > 0:
            return float(2.0 * self.bulk_modulus * self.piston_area ** 2 / self.chamber_volume)
        return 0.0

    def forces(
        self,
        z_left: float,
        z_right: float,
        z_dot_left: float = 0.0,
        z_dot_right: float = 0.0,
    ) -> tuple[float, float, float]:
        if not self.enabled:
            return 0.0, 0.0, 0.0
        k = self.effective_stiffness()
        c = self.c_hyd
        if abs(k) < 1e-9 and abs(c) < 1e-9:
            return 0.0, 0.0, 0.0
        dz = z_left - z_right
        dz_dot = z_dot_left - z_dot_right
        f_pair = k * dz + c * dz_dot
        f_left = -f_pair
        f_right = +f_pair
        M = f_right * (self.track / 2) - f_left * (self.track / 2)
        return f_left, f_right, M

    def dissipate(self, z_dot_left: float, z_dot_right: float, dt: float) -> float:
        """Accumulate orifice dissipation ≥ 0."""
        dz_dot = z_dot_left - z_dot_right
        dE = abs(self.c_hyd) * dz_dot ** 2 * max(dt, 0.0)
        self.E_dissipated += dE
        return dE


@dataclass
class DualAxleARB:
    """Front and rear independent ARBs (mechanical or hydraulic)."""

    front: MechanicalAntiRollBar | HydraulicAntiRollBar
    rear: MechanicalAntiRollBar | HydraulicAntiRollBar

    def axle_forces(
        self,
        z_s: list | tuple,
        z_s_dot: list | tuple,
    ) -> tuple[float, float, float, float]:
        """
        z_s = [FL, FR, RL, RR] corner heights.
        Returns F_FL, F_FR, F_RL, F_RR on body (up +).
        """
        fl, fr, m_f = self.front.forces(z_s[0], z_s[1], z_s_dot[0], z_s_dot[1])
        rl, rr, m_r = self.rear.forces(z_s[2], z_s[3], z_s_dot[2], z_s_dot[3])
        return fl, fr, rl, rr
