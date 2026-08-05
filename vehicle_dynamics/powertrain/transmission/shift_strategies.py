"""Shift request interpretation (manual / sequential / paddle)."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
import numpy as np


class ShiftStrategy(ABC):
    @abstractmethod
    def resolve(self, requested: int, current: int, n_forward: int) -> int:
        ...


@dataclass
class ManualStrategy(ShiftStrategy):
    """Absolute gear request (H-pattern)."""

    def resolve(self, requested: int, current: int, n_forward: int) -> int:
        if requested < 0:
            return -1
        return int(np.clip(requested, 0, n_forward))


@dataclass
class SequentialStrategy(ShiftStrategy):
    """
    requested: +1 up, -1 down, 0 hold, or absolute if |requested| > 1.
    """

    def resolve(self, requested: int, current: int, n_forward: int) -> int:
        if requested == 0:
            return current
        if abs(requested) > 1:
            if requested < 0:
                return -1
            return int(np.clip(requested, 0, n_forward))
        # Relative
        if current < 0 and requested > 0:
            return 1
        nxt = current + int(np.sign(requested))
        if nxt < 0:
            return -1 if current <= 0 else 0
        return int(np.clip(nxt, 0, n_forward))
