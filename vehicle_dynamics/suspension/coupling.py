"""
Phase 6.2 – Geometry coupling bridge (hardpoints → CornerState).

Camber is diagnostic only — do not feed into tire forces in this phase.
Toe is applied only via SuspensionInterface.effective_steer.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np

from .hardpoints import WishboneHardpoints, default_front_left, mirror_corner
from .wishbone import analyze
from .result import GeometryResult
from .wheel_rate import (
    SpringDamperParams,
    MotionRatioParams,
    compute_wheel_rate,
    WheelRateResult,
)
from .geometry_state import WheelGeometryState


@dataclass
class CornerConfig:
    hardpoints: WishboneHardpoints
    spring: SpringDamperParams = field(default_factory=SpringDamperParams)
    mr: MotionRatioParams = field(default_factory=MotionRatioParams)
    name: str = "corner"


@dataclass
class CornerState:
    name: str
    camber_rad: float
    toe_rad: float
    caster_rad: float
    kpi_rad: float
    scrub_radius: float
    trail: float
    roll_center_z: float
    installation_ratio: float
    motion_ratio: float
    Kw: float
    Cw: float
    Ks: float
    Cs: float

    @property
    def camber_deg(self) -> float:
        return float(np.degrees(self.camber_rad))

    @property
    def toe_deg(self) -> float:
        return float(np.degrees(self.toe_rad))

    def to_wheel_geometry_state(self) -> WheelGeometryState:
        return WheelGeometryState(
            camber_rad=self.camber_rad,
            toe_rad=self.toe_rad,
            kpi_rad=self.kpi_rad,
            caster_rad=self.caster_rad,
            scrub_radius=self.scrub_radius,
            trail=self.trail,
            roll_center_z=self.roll_center_z,
            installation_ratio=self.installation_ratio,
            motion_ratio=self.motion_ratio,
            Kw=self.Kw,
            Cw=self.Cw,
        )


def _geom_to_state(name: str, geom: GeometryResult, wr: WheelRateResult) -> CornerState:
    return CornerState(
        name=name,
        camber_rad=float(np.radians(geom.camber_deg)),
        toe_rad=float(np.radians(geom.toe_deg)),
        caster_rad=float(np.radians(geom.caster_deg)),
        kpi_rad=float(np.radians(geom.kpi_deg)),
        scrub_radius=geom.scrub_radius,
        trail=geom.trail,
        roll_center_z=geom.roll_center_z,
        installation_ratio=wr.installation_ratio,
        motion_ratio=wr.motion_ratio,
        Kw=wr.Kw,
        Cw=wr.Cw,
        Ks=wr.Ks,
        Cs=wr.Cs,
    )


class CornerSuspension:
    def __init__(self, config: CornerConfig):
        self.config = config
        self._geom = analyze(config.hardpoints)
        self._wr = compute_wheel_rate(config.spring, config.mr)

    def evaluate(self) -> CornerState:
        return _geom_to_state(self.config.name, self._geom, self._wr)

    def wheel_spring_force(self, wheel_compression: float) -> float:
        return self._wr.Kw * float(wheel_compression)

    def wheel_damper_force(self, wheel_velocity: float) -> float:
        return self._wr.Cw * float(wheel_velocity)


@dataclass
class VehicleSuspensionConfig:
    fl: CornerConfig = None
    fr: CornerConfig = None
    rl: CornerConfig = None
    rr: CornerConfig = None

    def __post_init__(self):
        if self.fl is None:
            self.fl = CornerConfig(
                hardpoints=default_front_left(),
                name="FL",
                mr=MotionRatioParams(1.0, "direct"),
            )
        if self.fr is None:
            self.fr = CornerConfig(
                hardpoints=mirror_corner(self.fl.hardpoints),
                spring=self.fl.spring,
                mr=self.fl.mr,
                name="FR",
            )
        if self.rl is None:
            self.rl = CornerConfig(
                hardpoints=default_front_left(),
                spring=SpringDamperParams(Ks=28000, Cs=1800),
                mr=MotionRatioParams(0.9, "pushrod"),
                name="RL",
            )
        if self.rr is None:
            self.rr = CornerConfig(
                hardpoints=mirror_corner(self.rl.hardpoints),
                spring=self.rl.spring,
                mr=self.rl.mr,
                name="RR",
            )


class CoupledSuspension:
    def __init__(self, config: VehicleSuspensionConfig = None):
        self.config = config or VehicleSuspensionConfig()
        self.corners = {
            "FL": CornerSuspension(self.config.fl),
            "FR": CornerSuspension(self.config.fr),
            "RL": CornerSuspension(self.config.rl),
            "RR": CornerSuspension(self.config.rr),
        }

    def evaluate_all(self) -> dict[str, CornerState]:
        return {k: c.evaluate() for k, c in self.corners.items()}

    def camber_toe_arrays(self) -> tuple[np.ndarray, np.ndarray]:
        order = ["FL", "FR", "RL", "RR"]
        states = [self.corners[k].evaluate() for k in order]
        return (
            np.array([s.camber_rad for s in states]),
            np.array([s.toe_rad for s in states]),
        )

    def wheel_rates(self) -> tuple[np.ndarray, np.ndarray]:
        order = ["FL", "FR", "RL", "RR"]
        states = [self.corners[k].evaluate() for k in order]
        return (
            np.array([s.Kw for s in states]),
            np.array([s.Cw for s in states]),
        )

    def vertical_forces(
        self,
        compression: np.ndarray,
        compression_rate: np.ndarray,
    ) -> np.ndarray:
        order = ["FL", "FR", "RL", "RR"]
        F = np.zeros(4)
        for i, k in enumerate(order):
            c = self.corners[k]
            F[i] = c.wheel_spring_force(compression[i]) + c.wheel_damper_force(
                compression_rate[i]
            )
        return F

    def ride_frequency_hz(self, sprung_mass_corner: float, corner: str = "FL") -> float:
        st = self.corners[corner].evaluate()
        return float(
            (1.0 / (2.0 * np.pi))
            * np.sqrt(st.Kw / max(sprung_mass_corner, 1.0))
        )
