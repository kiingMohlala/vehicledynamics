"""Modal analysis result containers."""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np


@dataclass
class ModeShape:
    index: int
    frequency_Hz: float
    omega_rad_s: float
    eigenvalue: float
    period_s: float
    shape: np.ndarray  # full ndof vector
    classification: str = ""
    mass_normalized: bool = True


@dataclass
class ModalResult:
    frequencies_Hz: np.ndarray
    omega: np.ndarray
    eigenvalues: np.ndarray
    mode_shapes: np.ndarray  # columns are eigenvectors (ndof × n_modes)
    modes: list[ModeShape] = field(default_factory=list)
    n_rigid_body: int = 0
    success: bool = True
    message: str = "ok"
    mass_type: str = "consistent"

    def mode(self, i: int) -> ModeShape:
        return self.modes[i]
