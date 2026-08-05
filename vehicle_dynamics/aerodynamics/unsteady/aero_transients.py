"""First-order lag on aero force buildup (relaxation-style)."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class AeroTransientFilter:
    """
    ẏ = (u - y) / τ

    Applies to downforce and drag channels.
    """

    tau_force: float = 0.08   # s
    tau_moment: float = 0.10

    def __post_init__(self) -> None:
        self._Fx = 0.0
        self._Fy = 0.0
        self._Fz_f = 0.0
        self._Fz_r = 0.0
        self._Mx = 0.0
        self._Mz = 0.0

    def reset(self, Fx=0.0, Fy=0.0, Fz_f=0.0, Fz_r=0.0, Mx=0.0, Mz=0.0) -> None:
        self._Fx, self._Fy = Fx, Fy
        self._Fz_f, self._Fz_r = Fz_f, Fz_r
        self._Mx, self._Mz = Mx, Mz

    def step(
        self,
        Fx: float,
        Fy: float,
        Fz_f: float,
        Fz_r: float,
        Mx: float,
        Mz: float,
        dt: float,
    ) -> tuple[float, float, float, float, float, float]:
        if dt <= 0 or self.tau_force <= 0:
            self._Fx, self._Fy = Fx, Fy
            self._Fz_f, self._Fz_r = Fz_f, Fz_r
            self._Mx, self._Mz = Mx, Mz
            return Fx, Fy, Fz_f, Fz_r, Mx, Mz

        a_f = 1.0 - np.exp(-dt / self.tau_force)
        a_m = 1.0 - np.exp(-dt / self.tau_moment)
        self._Fx += a_f * (Fx - self._Fx)
        self._Fy += a_f * (Fy - self._Fy)
        self._Fz_f += a_f * (Fz_f - self._Fz_f)
        self._Fz_r += a_f * (Fz_r - self._Fz_r)
        self._Mx += a_m * (Mx - self._Mx)
        self._Mz += a_m * (Mz - self._Mz)
        return self._Fx, self._Fy, self._Fz_f, self._Fz_r, self._Mx, self._Mz
