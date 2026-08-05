"""ESC activation logging helpers."""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np


@dataclass
class ESCDiagnostics:
    time: list = field(default_factory=list)
    active: list = field(default_factory=list)
    r_ref: list = field(default_factory=list)
    e_r: list = field(default_factory=list)
    beta: list = field(default_factory=list)
    Mz: list = field(default_factory=list)
    scale: list = field(default_factory=list)

    def log(self, t, diag: dict, scale: np.ndarray):
        self.time.append(float(t))
        self.active.append(bool(diag.get("active", False)))
        self.r_ref.append(float(diag.get("r_ref", 0.0)))
        self.e_r.append(float(diag.get("e_r", 0.0)))
        self.beta.append(float(diag.get("beta", 0.0)))
        self.Mz.append(float(diag.get("Mz", 0.0)))
        self.scale.append(np.asarray(scale, dtype=float).copy())

    def as_arrays(self) -> dict:
        return {
            "time": np.asarray(self.time),
            "active": np.asarray(self.active, dtype=bool),
            "r_ref": np.asarray(self.r_ref),
            "e_r": np.asarray(self.e_r),
            "beta": np.asarray(self.beta),
            "Mz": np.asarray(self.Mz),
            "scale": np.asarray(self.scale),
        }

    @property
    def activation_fraction(self) -> float:
        if not self.active:
            return 0.0
        return float(np.mean(self.active))
