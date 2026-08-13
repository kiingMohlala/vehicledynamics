"""Chassis / suspension structural load cases."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any
import numpy as np


@dataclass
class LoadCase:
    name: str
    forces: dict[str, np.ndarray] = field(default_factory=dict)  # node_tag -> Fx,Fy,Fz
    moments: dict[str, np.ndarray] = field(default_factory=dict)
    scale: float = 1.0
    meta: dict[str, Any] = field(default_factory=dict)

    def scaled(self) -> "LoadCase":
        f = {k: v * self.scale for k, v in self.forces.items()}
        m = {k: v * self.scale for k, v in self.moments.items()}
        return LoadCase(self.name, f, m, 1.0, self.meta)


class LoadCases:
    @staticmethod
    def bump(force: float = 15000.0) -> LoadCase:
        return LoadCase("bump", {"FL": np.array([0, 0, force]), "FR": np.array([0, 0, force]),
                                 "RL": np.array([0, 0, force]), "RR": np.array([0, 0, force])})

    @staticmethod
    def cornering(ay_g: float = 1.5, mass: float = 1400.0) -> LoadCase:
        F = mass * 9.81 * ay_g
        # lateral at contact patches
        return LoadCase(
            "cornering",
            {
                "FL": np.array([0, F * 0.3, 0]),
                "FR": np.array([0, F * 0.3, 0]),
                "RL": np.array([0, F * 0.2, 0]),
                "RR": np.array([0, F * 0.2, 0]),
            },
            meta={"ay_g": ay_g},
        )

    @staticmethod
    def braking(ax_g: float = 1.2, mass: float = 1400.0) -> LoadCase:
        F = mass * 9.81 * ax_g
        return LoadCase(
            "braking",
            {
                "FL": np.array([-F * 0.35, 0, 0]),
                "FR": np.array([-F * 0.35, 0, 0]),
                "RL": np.array([-F * 0.15, 0, 0]),
                "RR": np.array([-F * 0.15, 0, 0]),
            },
            meta={"ax_g": ax_g},
        )

    @staticmethod
    def acceleration(ax_g: float = 0.8, mass: float = 1400.0) -> LoadCase:
        F = mass * 9.81 * ax_g
        return LoadCase(
            "acceleration",
            {
                "RL": np.array([F * 0.5, 0, 0]),
                "RR": np.array([F * 0.5, 0, 0]),
            },
            meta={"ax_g": ax_g},
        )

    @staticmethod
    def aero_downforce(Fz: float = 4000.0) -> LoadCase:
        return LoadCase(
            "aero",
            {
                "front_aero": np.array([0, 0, -Fz * 0.4]),
                "rear_aero": np.array([0, 0, -Fz * 0.6]),
            },
            meta={"downforce": Fz},
        )

    @staticmethod
    def torsion_rig(force: float = 1000.0) -> LoadCase:
        """Equal-opposite vertical loads at diagonally opposite corners."""
        return LoadCase(
            "torsion",
            {
                "FL": np.array([0, 0, force]),
                "RR": np.array([0, 0, force]),
                "FR": np.array([0, 0, -force]),
                "RL": np.array([0, 0, -force]),
            },
        )

    @staticmethod
    def suspension_mount(force: float = 8000.0, corner: str = "FL") -> LoadCase:
        return LoadCase("suspension_mount", {corner: np.array([0, 0, force])})

    @staticmethod
    def engine_mount(force: float = 5000.0) -> LoadCase:
        return LoadCase("engine_mount", {"engine": np.array([0, 0, -force])})

    @staticmethod
    def combined(ay_g: float = 1.2, ax_g: float = 0.5, mass: float = 1400.0) -> LoadCase:
        c = LoadCases.cornering(ay_g, mass)
        b = LoadCases.braking(ax_g, mass)
        forces = {}
        for k in set(c.forces) | set(b.forces):
            forces[k] = c.forces.get(k, np.zeros(3)) + b.forces.get(k, np.zeros(3))
        return LoadCase("combined", forces, meta={"ay_g": ay_g, "ax_g": ax_g})
