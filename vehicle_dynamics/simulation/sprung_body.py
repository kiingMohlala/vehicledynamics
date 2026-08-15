"""
Phase 14.5 — Sprung-mass heave / pitch / roll dynamics.

Wheel loads emerge from suspension forces driven by body states,
not from algebraic quasi-static transfer.

States (body frame, small-angle):
  z, z_dot          heave (m), positive up from static ride
  theta, theta_dot  pitch (rad), positive nose-up
  phi, phi_dot      roll (rad), positive right-side down

Corner deflection from static equilibrium (+θ nose-up raises front):
  δ_FL = z + a·θ + (tf/2)·φ
  δ_FR = z + a·θ - (tf/2)·φ
  δ_RL = z - b·θ + (tr/2)·φ
  δ_RR = z - b·θ - (tr/2)·φ

With unsprung (14.7): δ = z_s_corner − z_u_corner

Suspension force on body (upward positive):
  F_i = -k_i·δ_i - c_i·δ̇_i

Wheel normal load (tire, positive down into contact):
  Fz_i = Fz_static_i + F_i + aero_share_i
  (F_i = force on body up; compression → F_i>0 → more tire load)

Body equations:
  m·z̈     = Σ F_i + Fz_aero_net
  Iθ·θ̈    = -a(F_FL+F_FR) + b(F_RL+F_RR) + m·ax·h_cg + M_aero_pitch
  Iφ·φ̈    = (tf/2)(F_FL-F_FR) + (tr/2)(F_RL-F_RR) + m·ay·h_cg + M_aero_roll
"""
from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np


@dataclass
class SprungBodyConfig:
    mass: float = 1100.0
    a: float = 1.25          # CG to front axle
    b: float = 1.45          # CG to rear axle
    track_f: float = 1.65
    track_r: float = 1.62
    h_cg: float = 0.40
    # Moments of inertia (sprung)
    I_theta: float = 1200.0  # pitch kg·m²
    I_phi: float = 400.0     # roll kg·m²
    # Per-axle spring/damper (N/m, N·s/m) — split L/R equally
    k_front: float = 28000.0
    k_rear: float = 32000.0
    c_front: float = 2500.0
    c_rear: float = 2800.0
    # Additional anti-roll (N·m/rad) distributed as vertical force couple
    roll_stiffness_front: float = 20000.0
    roll_stiffness_rear: float = 18000.0
    Fz_min: float = 50.0
    enabled: bool = True


@dataclass
class SprungBodyState:
    z: float = 0.0
    z_dot: float = 0.0
    theta: float = 0.0
    theta_dot: float = 0.0
    phi: float = 0.0
    phi_dot: float = 0.0
    # diagnostics
    Fz: np.ndarray = field(default_factory=lambda: np.zeros(4))
    F_susp: np.ndarray = field(default_factory=lambda: np.zeros(4))
    delta: np.ndarray = field(default_factory=lambda: np.zeros(4))
    E_spring: float = 0.0
    E_damp_dissipated: float = 0.0  # cumulative ∫ c·δ̇² dt
    residual_Fz: float = 0.0


class SprungBodyModel:
    """3-DOF sprung mass → four-corner Fz."""

    def __init__(self, cfg: SprungBodyConfig | None = None):
        self.cfg = cfg or SprungBodyConfig()
        self.state = SprungBodyState()
        self._E_damp = 0.0
        self._static_Fz = self._compute_static_Fz()

    def _compute_static_Fz(self) -> np.ndarray:
        m, a, b = self.cfg.mass, self.cfg.a, self.cfg.b
        L = a + b
        g = 9.81
        Fz_f = m * g * b / L
        Fz_r = m * g * a / L
        return np.array([Fz_f / 2, Fz_f / 2, Fz_r / 2, Fz_r / 2])

    def reset(self) -> None:
        self.state = SprungBodyState()
        self._E_damp = 0.0
        self._static_Fz = self._compute_static_Fz()
        self.state.Fz = self._static_Fz.copy()

    def corner_positions(self) -> tuple[np.ndarray, np.ndarray]:
        """Sprung corner height and velocity from static ( +up )."""
        cfg = self.cfg
        a, b = cfg.a, cfg.b
        tf, tr = cfg.track_f, cfg.track_r
        z, zd = self.state.z, self.state.z_dot
        th, thd = self.state.theta, self.state.theta_dot
        ph, phd = self.state.phi, self.state.phi_dot
        z_s = np.array([
            z + a * th + (tf / 2) * ph,
            z + a * th - (tf / 2) * ph,
            z - b * th + (tr / 2) * ph,
            z - b * th - (tr / 2) * ph,
        ])
        z_s_dot = np.array([
            zd + a * thd + (tf / 2) * phd,
            zd + a * thd - (tf / 2) * phd,
            zd - b * thd + (tr / 2) * phd,
            zd - b * thd - (tr / 2) * phd,
        ])
        return z_s, z_s_dot

    def step(
        self,
        *,
        ax: float,
        ay: float,
        dt: float,
        downforce_front: float = 0.0,
        downforce_rear: float = 0.0,
        M_aero_pitch: float = 0.0,
        M_aero_roll: float = 0.0,
        z_u: np.ndarray | None = None,
        z_u_dot: np.ndarray | None = None,
        Fz_contact: np.ndarray | None = None,
    ) -> SprungBodyState:
        cfg = self.cfg
        if not cfg.enabled:
            # fall back: quasi-static still available via caller
            self.state.Fz = self._static_Fz.copy()
            return self.state

        a, b = cfg.a, cfg.b
        tf, tr = cfg.track_f, cfg.track_r
        z, zd = self.state.z, self.state.z_dot
        th, thd = self.state.theta, self.state.theta_dot
        ph, phd = self.state.phi, self.state.phi_dot

        # Corner sprung positions
        z_s = np.array([
            z + a * th + (tf / 2) * ph,
            z + a * th - (tf / 2) * ph,
            z - b * th + (tr / 2) * ph,
            z - b * th - (tr / 2) * ph,
        ])
        z_s_dot = np.array([
            zd + a * thd + (tf / 2) * phd,
            zd + a * thd - (tf / 2) * phd,
            zd - b * thd + (tr / 2) * phd,
            zd - b * thd - (tr / 2) * phd,
        ])
        # Relative to unsprung (14.7) or ground-fixed (14.5: z_u=0)
        if z_u is None:
            z_u = np.zeros(4)
            z_u_dot = np.zeros(4)
        else:
            z_u = np.asarray(z_u, dtype=float)
            z_u_dot = np.zeros(4) if z_u_dot is None else np.asarray(z_u_dot, dtype=float)
        delta = z_s - z_u
        delta_dot = z_s_dot - z_u_dot

        k = np.array([cfg.k_front, cfg.k_front, cfg.k_rear, cfg.k_rear]) / 1.0  # per corner
        # axle rates given as axle total → half per corner
        k = np.array([cfg.k_front / 2, cfg.k_front / 2, cfg.k_rear / 2, cfg.k_rear / 2])
        c = np.array([cfg.c_front / 2, cfg.c_front / 2, cfg.c_rear / 2, cfg.c_rear / 2])

        # Spring-damper force on body (upward +)
        F_sd = -k * delta - c * delta_dot

        # Anti-roll: additional couple from roll angle
        # F_AR_front on left = +Kr_f * phi / tf , on right = -Kr_f * phi / tf
        if tf > 1e-6:
            Far_f = cfg.roll_stiffness_front * ph / tf
            F_sd[0] += Far_f
            F_sd[1] -= Far_f
        if tr > 1e-6:
            Far_r = cfg.roll_stiffness_rear * ph / tr
            F_sd[2] += Far_r
            F_sd[3] -= Far_r

        # Wheel normal loads: from tire contact (14.7) or suspension (14.5)
        if Fz_contact is not None:
            Fz = np.maximum(np.asarray(Fz_contact, dtype=float), cfg.Fz_min)
        else:
            Fz = self._static_Fz + F_sd
            Fz = np.maximum(Fz, cfg.Fz_min)

        # Body accelerations
        Fz_aero_net = downforce_front + downforce_rear  # upward reaction on body = -downforce
        # Net vertical force on sprung mass: suspension pushes up with -F_sd wait
        # F_sd is force on body upward. Static equilibrium: at delta=0, F_sd=0, weight
        # supported by preload (not in dynamic eq). Dynamic:
        z_ddot = float(np.sum(F_sd) - Fz_aero_net) / cfg.mass
        # Pitch: suspension moments + inertial load-transfer moment
        M_susp_th = +a * (F_sd[0] + F_sd[1]) - b * (F_sd[2] + F_sd[3])
        theta_ddot = (M_susp_th + cfg.mass * ax * cfg.h_cg + M_aero_pitch) / cfg.I_theta
        # Roll
        M_susp_ph = (tf / 2) * (F_sd[0] - F_sd[1]) + (tr / 2) * (F_sd[2] - F_sd[3])
        phi_ddot = (M_susp_ph + cfg.mass * ay * cfg.h_cg + M_aero_roll) / cfg.I_phi

        # Integrate (semi-implicit Euler)
        zd_n = zd + z_ddot * dt
        z_n = z + zd_n * dt
        thd_n = thd + theta_ddot * dt
        th_n = th + thd_n * dt
        phd_n = phd + phi_ddot * dt
        ph_n = ph + phd_n * dt

        # Energy
        E_spring = 0.5 * float(np.sum(k * delta ** 2))
        dE_damp = float(np.sum(c * delta_dot ** 2)) * dt  # dissipated ≥ 0
        self._E_damp += max(dE_damp, 0.0)

        expected = float(np.sum(self._static_Fz) + downforce_front + downforce_rear)
        residual = float(np.sum(Fz) - expected)  # →0 after heave settles under aero

        self.state = SprungBodyState(
            z=float(z_n),
            z_dot=float(zd_n),
            theta=float(th_n),
            theta_dot=float(thd_n),
            phi=float(ph_n),
            phi_dot=float(phd_n),
            Fz=Fz,
            F_susp=F_sd,
            delta=delta,
            E_spring=E_spring,
            E_damp_dissipated=self._E_damp,
            residual_Fz=residual,
        )
        return self.state
