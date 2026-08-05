"""Aero design variables and bounds."""

from __future__ import annotations

from dataclasses import dataclass, fields
import numpy as np


@dataclass
class DesignVector:
    front_wing_angle: float = 0.10      # rad
    rear_wing_angle: float = 0.12
    wing_span_scale: float = 1.0        # relative
    chord_scale: float = 1.0
    gurney_mm: float = 0.0              # mm
    diffuser_angle: float = 0.15        # rad
    diffuser_height: float = 0.10       # m
    splitter_length: float = 0.20       # m
    h_front: float = 0.08               # m
    h_rear: float = 0.10
    drs_schedule: float = 0.0           # 0..1 fraction of lap DRS open

    def as_array(self) -> np.ndarray:
        return np.array([getattr(self, f.name) for f in fields(self)], dtype=float)

    @classmethod
    def from_array(cls, x: np.ndarray) -> "DesignVector":
        names = [f.name for f in fields(cls)]
        return cls(**{n: float(v) for n, v in zip(names, x)})

    @property
    def rake(self) -> float:
        return self.h_rear - self.h_front


@dataclass
class DesignBounds:
    low: DesignVector
    high: DesignVector

    def clip(self, d: DesignVector) -> DesignVector:
        lo, hi = self.low.as_array(), self.high.as_array()
        x = np.clip(d.as_array(), lo, hi)
        return DesignVector.from_array(x)

    def random(self, rng: np.random.Generator) -> DesignVector:
        lo, hi = self.low.as_array(), self.high.as_array()
        x = rng.uniform(lo, hi)
        return DesignVector.from_array(x)

    def n_dim(self) -> int:
        return len(fields(DesignVector))


def default_bounds() -> DesignBounds:
    return DesignBounds(
        low=DesignVector(
            front_wing_angle=0.02, rear_wing_angle=0.02, wing_span_scale=0.8,
            chord_scale=0.8, gurney_mm=0.0, diffuser_angle=0.05,
            diffuser_height=0.04, splitter_length=0.05,
            h_front=0.04, h_rear=0.05, drs_schedule=0.0,
        ),
        high=DesignVector(
            front_wing_angle=0.25, rear_wing_angle=0.30, wing_span_scale=1.2,
            chord_scale=1.2, gurney_mm=15.0, diffuser_angle=0.30,
            diffuser_height=0.18, splitter_length=0.45,
            h_front=0.14, h_rear=0.16, drs_schedule=1.0,
        ),
    )
