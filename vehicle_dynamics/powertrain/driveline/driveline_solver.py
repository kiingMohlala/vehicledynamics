"""
Lumped torsional driveline solver.

Topology (2-DOF relative model + half-shafts):

  Engine torque ──► [J_gear] ── mesh/backlash ──► [propshaft k,c]
                         │                              │
                    ω_gear                           θ_shaft
                         │                              │
                         └──────── T_shaft ─────────────┘
                                        │
                                   [diff open split]
                                    /            \\
                             halfshaft L      halfshaft R
                                    \\            /
                                 [J_wheel L]  [J_wheel R]
                                    │            │
                               T_load_L      T_load_R

When enabled=False, pass-through: equal split of input torque (rigid regression).
"""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .shaft import ElasticShaft
from .halfshaft import HalfShaftPair
from .backlash import Backlash
from .gear_mesh import GearMesh
from .torsional_mass import TorsionalInertia
from .wheel_inertia import WheelInertia
from .driveline_state import AdvancedDrivelineState


@dataclass
class DrivelineConfig:
    enabled: bool = True
    # Propshaft
    shaft_stiffness: float = 12000.0
    shaft_damping: float = 40.0
    shaft_max_torque: float = 8000.0
    # Half-shafts
    halfshaft_stiffness: float = 8000.0
    halfshaft_damping: float = 25.0
    # Backlash
    backlash_deg: float = 0.4
    # Gear mesh
    mesh_stiffness: float = 5.0e4
    mesh_damping: float = 20.0
    mesh_ripple: float = 0.0
    # Inertias
    J_gear: float = 0.12
    J_diff: float = 0.08
    J_wheel: float = 0.9
    J_rotor: float = 0.15
    radius: float = 0.32
    # Integration
    max_omega: float = 3000.0  # rad/s clamp


class DrivelineSolver:
    def __init__(self, config: DrivelineConfig | None = None):
        self.cfg = config or DrivelineConfig()
        self.prop = ElasticShaft(
            self.cfg.shaft_stiffness,
            self.cfg.shaft_damping,
            self.cfg.shaft_max_torque,
        )
        self.halfs = HalfShaftPair(
            k_left=self.cfg.halfshaft_stiffness,
            k_right=self.cfg.halfshaft_stiffness,
            c_left=self.cfg.halfshaft_damping,
            c_right=self.cfg.halfshaft_damping,
        )
        self.backlash = Backlash.from_degrees(self.cfg.backlash_deg)
        self.mesh = GearMesh(
            self.cfg.mesh_stiffness,
            self.cfg.mesh_damping,
            self.cfg.mesh_ripple,
        )
        self.J_gear = TorsionalInertia(self.cfg.J_gear, "gear")
        self.J_diff = TorsionalInertia(self.cfg.J_diff, "diff")
        self.wheel = WheelInertia(
            self.cfg.J_wheel, self.cfg.J_rotor, self.cfg.radius
        )

        # States
        self.theta_shaft = 0.0       # propshaft twist (gear relative to diff)
        self.omega_gear = 0.0
        self.omega_diff = 0.0
        self.theta_hs_L = 0.0
        self.theta_hs_R = 0.0
        self.omega_w_L = 0.0
        self.omega_w_R = 0.0
        self.theta_mesh = 0.0

        self._peak_T = 0.0
        self.state = AdvancedDrivelineState()

    def reset(self) -> None:
        self.theta_shaft = 0.0
        self.omega_gear = 0.0
        self.omega_diff = 0.0
        self.theta_hs_L = 0.0
        self.theta_hs_R = 0.0
        self.omega_w_L = 0.0
        self.omega_w_R = 0.0
        self.theta_mesh = 0.0
        self._peak_T = 0.0
        self.state = AdvancedDrivelineState(enabled=self.cfg.enabled)

    def _oscillation_freq(self) -> float:
        """Natural freq estimate for gear–diff mode: ωn = sqrt(k/μ)/2π."""
        k = self.cfg.shaft_stiffness
        mu = 1.0 / (1.0 / max(self.cfg.J_gear, 1e-9) + 1.0 / max(self.cfg.J_diff, 1e-9))
        if k <= 0 or mu <= 0:
            return 0.0
        return float(np.sqrt(k / mu) / (2.0 * np.pi))

    def step(
        self,
        engine_torque: float,
        wheel_load_left: float = 0.0,
        wheel_load_right: float = 0.0,
        dt: float = 0.001,
        *,
        omega_wheel_left: float | None = None,
        omega_wheel_right: float | None = None,
    ) -> AdvancedDrivelineState:
        """
        Integrate one timestep.

        engine_torque: torque into gearbox output / propshaft input [N·m]
        wheel_load_*: opposing torque at each wheel (tire / brake) [N·m]
        omega_wheel_*: optional external wheel speeds (if provided, used as kinematic constraint soft update)
        """
        cfg = self.cfg
        dt = float(max(dt, 1e-6))
        T_in = float(engine_torque)
        T_load_L = float(wheel_load_left)
        T_load_R = float(wheel_load_right)

        if not cfg.enabled:
            # Rigid pass-through — Phase 10.2 regression
            T_L = 0.5 * T_in
            T_R = 0.5 * T_in
            self.state = AdvancedDrivelineState(
                torque_left=T_L,
                torque_right=T_R,
                torque_input=T_in,
                axle_speed=0.5 * (self.omega_w_L + self.omega_w_R),
                enabled=False,
            )
            return self.state

        # Optional soft lock to external wheel speeds
        if omega_wheel_left is not None:
            self.omega_w_L = float(omega_wheel_left)
        if omega_wheel_right is not None:
            self.omega_w_R = float(omega_wheel_right)

        # --- Gear mesh + backlash on propshaft twist ---
        bl = self.backlash.evaluate(self.theta_shaft)
        theta_eff = self.backlash.effective_angle(self.theta_shaft)
        omega_rel = self.omega_gear - self.omega_diff

        # Propshaft elastic torque (through backlash dead-zone)
        if bl.engaged:
            T_shaft = self.prop.torque(theta_eff, omega_rel)
        else:
            T_shaft = 0.0

        # Gear mesh (between engine input and gear inertia — simplified on theta_mesh)
        self.theta_mesh += omega_rel * dt * 0.1  # slow state for ripple
        T_mesh = self.mesh.torque(self.theta_mesh, omega_rel * 0.05)

        # Gear inertia: T_in - T_shaft - mesh damping contribution
        T_gear_net = T_in - T_shaft - 0.05 * T_mesh
        a_gear = self.J_gear.accel(T_gear_net)

        # Diff carrier inertia: T_shaft - halfshaft torques
        hs = self.halfs.evaluate(
            self.theta_hs_L,
            self.omega_diff - self.omega_w_L,
            self.theta_hs_R,
            self.omega_diff - self.omega_w_R,
        )
        T_hs_L = hs.torque_left
        T_hs_R = hs.torque_right
        T_diff_net = T_shaft - T_hs_L - T_hs_R
        a_diff = self.J_diff.accel(T_diff_net)

        # Wheels
        a_w_L = self.wheel.accel(T_hs_L - T_load_L)
        a_w_R = self.wheel.accel(T_hs_R - T_load_R)

        # Integrate (semi-implicit Euler)
        self.omega_gear = float(np.clip(self.omega_gear + a_gear * dt, -cfg.max_omega, cfg.max_omega))
        self.omega_diff = float(np.clip(self.omega_diff + a_diff * dt, -cfg.max_omega, cfg.max_omega))
        self.omega_w_L = float(np.clip(self.omega_w_L + a_w_L * dt, -cfg.max_omega, cfg.max_omega))
        self.omega_w_R = float(np.clip(self.omega_w_R + a_w_R * dt, -cfg.max_omega, cfg.max_omega))

        self.theta_shaft += (self.omega_gear - self.omega_diff) * dt
        self.theta_hs_L += (self.omega_diff - self.omega_w_L) * dt
        self.theta_hs_R += (self.omega_diff - self.omega_w_R) * dt

        # Soft twist limits (numerical safety)
        self.theta_shaft = float(np.clip(self.theta_shaft, -1.0, 1.0))
        self.theta_hs_L = float(np.clip(self.theta_hs_L, -1.0, 1.0))
        self.theta_hs_R = float(np.clip(self.theta_hs_R, -1.0, 1.0))

        energy = (
            self.prop.energy(theta_eff)
            + 0.5 * cfg.halfshaft_stiffness * self.theta_hs_L ** 2
            + 0.5 * cfg.halfshaft_stiffness * self.theta_hs_R ** 2
        )
        self._peak_T = max(self._peak_T, abs(T_shaft), abs(T_hs_L), abs(T_hs_R))

        self.state = AdvancedDrivelineState(
            torque_left=T_hs_L,
            torque_right=T_hs_R,
            torque_input=T_in,
            axle_speed=0.5 * (self.omega_w_L + self.omega_w_R),
            delta_omega=self.omega_w_L - self.omega_w_R,
            efficiency=1.0,
            engine_speed=self.omega_gear,
            gearbox_speed=self.omega_gear,
            propshaft_speed=self.omega_diff,
            wheel_speed_left=self.omega_w_L,
            wheel_speed_right=self.omega_w_R,
            shaft_twist=self.theta_shaft,
            halfshaft_twist_L=self.theta_hs_L,
            halfshaft_twist_R=self.theta_hs_R,
            mesh_theta=self.theta_mesh,
            torsional_energy=energy,
            backlash_engaged=bl.engaged,
            backlash_side=bl.side,
            peak_torque=self._peak_T,
            oscillation_freq_hz=self._oscillation_freq(),
            enabled=True,
        )
        return self.state
