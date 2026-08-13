"""Vehicle geometry parameters."""
from __future__ import annotations
from dataclasses import dataclass


@dataclass
class GeometryConfig:
    wheelbase_m: float = 2.70
    track_front_m: float = 1.55
    track_rear_m: float = 1.55
    a_m: float = 1.20          # CG to front axle
    b_m: float = 1.50          # CG to rear axle
    h_cg_m: float = 0.50
    overall_length_m: float = 4.50
    overall_width_m: float = 1.90
    overall_height_m: float = 1.20
    wheel_radius_m: float = 0.32

    def __post_init__(self) -> None:
        L = self.a_m + self.b_m
        if abs(L - self.wheelbase_m) > 1e-6:
            self.wheelbase_m = L

    @property
    def L(self) -> float:
        return self.a_m + self.b_m
