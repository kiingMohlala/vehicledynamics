"""Torque vectoring diagnostics."""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np


@dataclass
class TVDiagnostics:
    time: list = field(default_factory=list)
    active: list = field(default_factory=list)
    r_ref: list = field(default_factory=list)
    e_r: list = field(default_factory=list)
    rear_delta_T: list = field(default_factory=list)
    T_drive: list = field(default_factory=list)

    def log(self, t: float, diag: dict):
        self.time.append(float(t))
        self.active.append(bool(diag.get("active", False)))
        self.r_ref.append(float(diag.get("r_ref", 0.0)))
        self.e_r.append(float(diag.get("e_r", 0.0)))
        self.rear_delta_T.append(float(diag.get("rear_delta_T", 0.0)))
        self.T_drive.append(np.asarray(diag.get("T_drive", np.zeros(4)), dtype=float).copy())

    @property
    def activation_fraction(self) -> float:
        if not self.active:
            return 0.0
        return float(np.mean(self.active))
