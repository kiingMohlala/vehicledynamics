"""Beam element definition."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np
from .node import Node
from .material import Material
from .section import Section


@dataclass
class BeamElement:
    """
    Two-node Euler-Bernoulli beam in 3D.

    Local axes:
      x : along beam (node i → node j)
      y, z : principal bending axes of the section
    """

    id: int
    node_i: Node
    node_j: Node
    material: Material
    section: Section
    tag: str = ""  # e.g. "main_hoop", "door_bar"

    def length(self) -> float:
        d = self.node_j.coords() - self.node_i.coords()
        L = float(np.linalg.norm(d))
        if L < 1e-12:
            raise ValueError(
                f"Zero-length beam element id={self.id} "
                f"(nodes {self.node_i.id} and {self.node_j.id})"
            )
        return L

    def direction(self) -> np.ndarray:
        d = self.node_j.coords() - self.node_i.coords()
        n = np.linalg.norm(d)
        if n < 1e-12:
            raise ValueError(f"Zero-length beam {self.id}")
        return d / n

    def mass(self) -> float:
        """Element mass [kg] = rho * A * L."""
        return self.material.rho * self.section.A * self.length()
