"""
Phase 6.2 – Geometry coupling bridge.

Connects standalone kinematics (6.0) and wheel-rate (6.1) to per-corner
quantities the dual-track model can consume:

  camber, toe, installation ratio, Kw, Cw

Vertical spring forces and full 3D tire orientation are prepared here;
force injection into DualTrackVehicle remains opt-in so the Phase 5
baseline is unchanged unless explicitly enabled.
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


@dataclass
class CornerConfig:
    """One suspension corner: geometry + spring/damper + MR layout."""
    hardpoints: WishboneHardpoints
    spring: SpringDamperParams = field(default_factory=SpringDamperParams)
    mr: MotionRatioParams = field(default_factory=MotionRatioParams)
    name: str = "corner"


@dataclass
class CornerState:
    """Coupled outputs for one wheel at the current configuration."""
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
    """
    Evaluates geometry + wheel rates for one corner.

    Static for Phase 6.2 (design ride height). Travel-dependent MR/camber
    curves arrive in 6.3–6.4.
    """

    def __init__(self, config: CornerConfig):
        self.config = config
        self._geom = analyze(config.hardpoints)
        self._wr = compute_wheel_rate(config.spring, config.mr)

    def evaluate(self) -> CornerState:
        return _geom_to_state(self.config.name, self._geom, self._wr)

    def wheel_spring_force(self, wheel_compression: float) -> float:
        """Vertical force at wheel from spring: F = Kw * z_wheel."""
        return self._wr.Kw * float(wheel_compression)

    def wheel_damper_force(self, wheel_velocity: float) -> float:
        """Vertical damping force at wheel: F = Cw * zdot_wheel."""
        return self._wr.Cw * float(wheel_velocity)


@dataclass
class VehicleSuspensionConfig:
    """Four corners. Defaults: mirrored L/R, same spring rates."""
    fl: CornerConfig = None
    fr: CornerConfig = None
    rl: CornerConfig = None
    rr: CornerConfig = None

    def __post_init__(self):
        if self.fl is None:
            hp = default_front_left()
            self.fl = CornerConfig(hardpoints=hp, name="FL",
                                  mr=MotionRatioParams(1.0, "direct"))
        if self.fr is None:
            self.fr = CornerConfig(
                hardpoints=mirror_corner(self.fl.hardpoints),
                spring=self.fl.spring,
                mr=self.fl.mr,
                name="FR",
            )
        if self.rl is None:
            # simple rear: copy front geometry shifted (illustrative)
            hp = default_front_left()
            self.rl = CornerConfig(
                hardpoints=hp,
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
    """
    Four-corner coupling facade for the vehicle model.

    Provides:
      - per-wheel camber / toe (for tire orientation)
      - per-wheel Kw / Cw (for vertical force path)
      - ride-frequency estimate from unsprung-ignored quarter-car
    """

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
        """Return camber[4], toe[4] in radians, order FL,FR,RL,RR."""
        order = ["FL", "FR", "RL", "RR"]
        states = [self.corners[k].evaluate() for k in order]
        camber = np.array([s.camber_rad for s in states])
        toe = np.array([s.toe_rad for s in states])
        return camber, toe

    def wheel_rates(self) -> tuple[np.ndarray, np.ndarray]:
        """Kw[4], Cw[4]."""
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
        """
        Per-wheel vertical suspension force (positive upward on body).
        compression: wheel bump travel [m], shape (4,)
        """
        order = ["FL", "FR", "RL", "RR"]
        F = np.zeros(4)
        for i, k in enumerate(order):
            c = self.corners[k]
            F[i] = c.wheel_spring_force(compression[i]) + c.wheel_damper_force(
                compression_rate[i]
            )
        return F

    def ride_frequency_hz(self, sprung_mass_corner: float, corner: str = "FL") -> float:
        """
        Undamped ride frequency for one corner (quarter-car, no tire):
          f = (1/2π) * sqrt(Kw / m_corner)
        """
        st = self.corners[corner].evaluate()
        return float((1.0 / (2.0 * np.pi)) * np.sqrt(st.Kw / max(sprung_mass_corner, 1.0)))


# ---- tire orientation helpers (camber / toe → slip frame) ----

def apply_toe_to_delta(delta_steer: float, toe: float) -> float:
    """Effective steer angle including static toe."""
    return float(delta_steer + toe)


def camber_lateral_force(
    camber_rad: float,
    Fz: float,
    Cy_camber: float = 1000.0,
) -> float:
    """
    Simple linear camber thrust: Fy_γ ≈ Cγ * γ  (scaled mildly by load).
    Cγ default is conservative; replace with tire-model camber stiffness later.
    """
    # normalize around nominal Fz=4000 N
    scale = float(Fz) / 4000.0
    return float(Cy_camber * scale * camber_rad)
