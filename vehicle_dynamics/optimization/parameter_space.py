"""Parameter space: collection of design variables + sample rows."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterator
import numpy as np

from .design_variables import DesignVariable


@dataclass
class ParameterSpace:
    variables: list[DesignVariable]
    samples: np.ndarray = field(default_factory=lambda: np.zeros((0, 0)))  # shape (n, n_vars)

    @property
    def n_vars(self) -> int:
        return len(self.variables)

    @property
    def n_samples(self) -> int:
        return 0 if self.samples.size == 0 else int(self.samples.shape[0])

    def names(self) -> list[str]:
        return [v.name for v in self.variables]

    def as_dicts(self) -> list[dict[str, float]]:
        rows = []
        for i in range(self.n_samples):
            rows.append({v.name: float(self.samples[i, j]) for j, v in enumerate(self.variables)})
        return rows

    def __iter__(self) -> Iterator[dict[str, float]]:
        yield from self.as_dicts()
