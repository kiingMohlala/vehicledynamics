"""Simple aero map with multilinear interpolation over (V, hf, hr, yaw)."""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np

from .coefficients import AeroConfig, AeroCoefficients
from .ride_height import RideHeightState, ride_height_factors


@dataclass
class AeroMap:
    """
    Dense coefficient samples for offline lookup.
    Axes: speed [m/s], h_front [m], h_rear [m], yaw [rad]
    Stored as dict of grids for Cl_f, Cl_r, Cd.
    """

    speeds: np.ndarray
    h_fronts: np.ndarray
    h_rears: np.ndarray
    yaws: np.ndarray
    Cl_front: np.ndarray  # shape (nv, nhf, nhr, ny)
    Cl_rear: np.ndarray
    Cd: np.ndarray
    cfg: AeroConfig = field(default_factory=AeroConfig)

    def evaluate(
        self,
        speed: float,
        h_front: float,
        h_rear: float,
        yaw: float = 0.0,
    ) -> AeroCoefficients:
        """Nearest-neighbor sample (sufficient for Phase 9.0 maps)."""
        i = int(np.argmin(np.abs(self.speeds - speed)))
        j = int(np.argmin(np.abs(self.h_fronts - h_front)))
        k = int(np.argmin(np.abs(self.h_rears - h_rear)))
        m = int(np.argmin(np.abs(self.yaws - yaw)))
        return AeroCoefficients(
            Cd=float(self.Cd[i, j, k, m]),
            Cl_front=float(self.Cl_front[i, j, k, m]),
            Cl_rear=float(self.Cl_rear[i, j, k, m]),
            Cy_beta=self.cfg.coeffs.Cy_beta,
            Cm_pitch=self.cfg.coeffs.Cm_pitch,
            Cn_yaw=self.cfg.coeffs.Cn_yaw,
        )


def build_default_map(cfg: AeroConfig | None = None) -> AeroMap:
    cfg = cfg or AeroConfig()
    speeds = np.array([0.0, 20.0, 40.0, 60.0, 80.0])
    hfs = np.array([0.04, 0.06, 0.08, 0.10, 0.12])
    hrs = np.array([0.05, 0.08, 0.10, 0.12, 0.15])
    yaws = np.array([-0.15, 0.0, 0.15])

    shape = (len(speeds), len(hfs), len(hrs), len(yaws))
    Cl_f = np.zeros(shape)
    Cl_r = np.zeros(shape)
    Cd = np.zeros(shape)

    for i, v in enumerate(speeds):
        for j, hf in enumerate(hfs):
            for k, hr in enumerate(hrs):
                for m, yaw in enumerate(yaws):
                    st = RideHeightState(h_front=hf, h_rear=hr, yaw_rad=yaw)
                    fac = ride_height_factors(st, cfg)
                    Cl_f[i, j, k, m] = cfg.coeffs.Cl_front * fac["Cl_front"]
                    Cl_r[i, j, k, m] = cfg.coeffs.Cl_rear * fac["Cl_rear"]
                    Cd[i, j, k, m] = cfg.coeffs.Cd * fac["Cd"]

    return AeroMap(
        speeds=speeds,
        h_fronts=hfs,
        h_rears=hrs,
        yaws=yaws,
        Cl_front=Cl_f,
        Cl_rear=Cl_r,
        Cd=Cd,
        cfg=cfg,
    )
