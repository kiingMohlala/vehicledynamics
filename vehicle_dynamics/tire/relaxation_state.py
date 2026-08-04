"""Relaxation filter state."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class RelaxationState:
    kappa_eff: float = 0.0
    alpha_eff: float = 0.0

    def copy(self) -> "RelaxationState":
        return RelaxationState(self.kappa_eff, self.alpha_eff)
