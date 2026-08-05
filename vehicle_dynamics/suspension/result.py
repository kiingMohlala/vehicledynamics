"""Suspension geometry result container."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class GeometryResult:
    camber_deg: float
    toe_deg: float
    caster_deg: float
    kpi_deg: float
    scrub_radius: float
    trail: float
    roll_center_z: float
    instant_center_y: float
    instant_center_z: float
    swing_arm_length: float
    upper_arm_length: float
    lower_arm_length: float

    def summary(self) -> str:
        lines = [
            f"Camber:          {self.camber_deg:8.3f} deg",
            f"Toe:             {self.toe_deg:8.3f} deg",
            f"Caster:          {self.caster_deg:8.3f} deg",
            f"KPI:             {self.kpi_deg:8.3f} deg",
            f"Scrub radius:    {self.scrub_radius:8.4f} m",
            f"Trail:           {self.trail:8.4f} m",
            f"Roll center z:   {self.roll_center_z:8.4f} m",
            f"IC (y,z):        ({self.instant_center_y:.4f}, {self.instant_center_z:.4f}) m",
            f"Swing arm:       {self.swing_arm_length:8.4f} m",
            f"Upper arm len:   {self.upper_arm_length:8.4f} m",
            f"Lower arm len:   {self.lower_arm_length:8.4f} m",
        ]
        return "\n".join(lines)
