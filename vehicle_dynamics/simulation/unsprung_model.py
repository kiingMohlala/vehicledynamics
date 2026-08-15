"""
Phase 14.7 — Unsprung mass & wheel-hop dynamics.

Road → tire (k_t, c_t) → unsprung mass → suspension (k, c) → sprung body

Four independent wheel vertical DOFs. Dynamic contact Fz feeds Dugoff.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np


@dataclass
class UnsprungConfig:
    # Per-corner unsprung mass (kg) — front/rear pairs
    m_u_front: float = 40.0
    m_u_rear: float = 45.0
    # Tire vertical stiffness / damping
    k_tire_front: float = 220000.0  # N/m
    k_tire_rear: float = 220000.0
    c_tire_front: float = 200.0     # N·s/m
    c_tire_rear: float = 200.0
    Fz_min: float = 50.0
    enabled: bool = True


@dataclass
class UnsprungState:
    z_u: np.ndarray = field(default_factory=lambda: np.zeros(4))
    z_u_dot: np.ndarray = field(default_factory=lambda: np.zeros(4))
    Fz: np.ndarray = field(default_factory=lambda: np.zeros(4))  # contact loads
    F_tire: np.ndarray = field(default_factory=lambda: np.zeros(4))
    F_susp_on_u: np.ndarray = field(default_factory=lambda: np.zeros(4))
    E_tire_spring: float = 0.0
    E_tire_damp: float = 0.0  # cumulative dissipation ≥ 0
    road_z: np.ndarray = field(default_factory=lambda: np.zeros(4))


class UnsprungModel:
    """Four-corner unsprung vertical dynamics."""

    def __init__(self, cfg: UnsprungConfig | None = None):
        self.cfg = cfg or UnsprungConfig()
        self.state = UnsprungState()
        self._E_damp = 0.0
        self._static_Fz = np.zeros(4)

    def set_static_Fz(self, Fz: np.ndarray) -> None:
        self._static_Fz = np.asarray(Fz, dtype=float).copy()

    def reset(self, static_Fz: np.ndarray | None = None) -> None:
        if static_Fz is not None:
            self._static_Fz = np.asarray(static_Fz, dtype=float).copy()
        self.state = UnsprungState()
        self.state.Fz = self._static_Fz.copy()
        self._E_damp = 0.0

    def step(
        self,
        *,
        z_s: np.ndarray,
        z_s_dot: np.ndarray,
        k_susp: np.ndarray,
        c_susp: np.ndarray,
        road_z: np.ndarray,
        road_z_dot: np.ndarray | None = None,
        dt: float,
        F_roll_extra: np.ndarray | None = None,
    ) -> UnsprungState:
        """
        z_s, z_s_dot: corner sprung positions/velocities from static (m, m/s), +up
        k_susp, c_susp: per-corner suspension rates
        road_z: road height at each wheel from static plane (+up)
        Returns state with contact Fz for Dugoff.
        """
        cfg = self.cfg
        if not cfg.enabled:
            # Pass-through: Fz from suspension only (14.5 behaviour approximated)
            self.state.Fz = self._static_Fz.copy()
            return self.state

        z_u = self.state.z_u.copy()
        zud = self.state.z_u_dot.copy()
        road_z = np.asarray(road_z, dtype=float)
        rzd = np.zeros(4) if road_z_dot is None else np.asarray(road_z_dot, dtype=float)

        m_u = np.array([
            cfg.m_u_front, cfg.m_u_front, cfg.m_u_rear, cfg.m_u_rear
        ])
        k_t = np.array([
            cfg.k_tire_front, cfg.k_tire_front, cfg.k_tire_rear, cfg.k_tire_rear
        ])
        c_t = np.array([
            cfg.c_tire_front, cfg.c_tire_front, cfg.c_tire_rear, cfg.c_tire_rear
        ])

        # Suspension deflection (extension + when body above wheel vs static)
        delta_s = z_s - z_u
        delta_s_dot = z_s_dot - zud
        # Force on unsprung from suspension (Newton 3: opposite of on body)
        # F_susp_on_body = -k*δ - c*δ̇
        # F_susp_on_u    = +k*δ + c*δ̇
        F_susp_on_u = k_susp * delta_s + c_susp * delta_s_dot
        if F_roll_extra is not None:
            # anti-roll already applied as on-body forces; opposite on unsprung
            F_susp_on_u = F_susp_on_u - np.asarray(F_roll_extra, dtype=float)

        # Tire: compression when z_u < road_z → force up on unsprung
        delta_t = z_u - road_z
        delta_t_dot = zud - rzd
        F_tire_on_u = -k_t * delta_t - c_t * delta_t_dot  # +up on wheel when compressed

        # Unsprung acceleration
        # m_u * z_u_ddot = F_susp_on_u + F_tire_on_u
        zu_ddot = (F_susp_on_u + F_tire_on_u) / np.maximum(m_u, 1e-6)

        # Integrate
        zud_n = zud + zu_ddot * dt
        z_u_n = z_u + zud_n * dt

        # Contact load for Dugoff (down into road +)
        # Additional contact force = F_tire_on_u when positive compression contribution
        Fz = self._static_Fz + F_tire_on_u
        Fz = np.maximum(Fz, cfg.Fz_min)

        # Energy
        E_ts = 0.5 * float(np.sum(k_t * delta_t ** 2))
        dE = float(np.sum(c_t * delta_t_dot ** 2)) * dt
        self._E_damp += max(dE, 0.0)

        self.state = UnsprungState(
            z_u=z_u_n,
            z_u_dot=zud_n,
            Fz=Fz,
            F_tire=F_tire_on_u,
            F_susp_on_u=F_susp_on_u,
            E_tire_spring=E_ts,
            E_tire_damp=self._E_damp,
            road_z=road_z.copy(),
        )
        return self.state
