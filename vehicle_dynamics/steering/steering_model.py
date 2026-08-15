"""
Phase 14.9.1 — Steering runtime model.

command → rate limit → angle limit → Ackermann → δ_FL / δ_FR
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math

from vehicle_dynamics.steering.steering_config import SteeringConfig


@dataclass
class SteeringState:
    command: float = 0.0          # requested road-wheel equivalent (rad)
    actual: float = 0.0           # rate-limited, clamped centreline angle (rad)
    delta_fl: float = 0.0
    delta_fr: float = 0.0
    delta_rl: float = 0.0
    delta_rr: float = 0.0


class SteeringModel:
    """Rate-limited front steering with optional Ackermann geometry."""

    def __init__(self, cfg: SteeringConfig | None = None):
        self.cfg = cfg or SteeringConfig()
        self.state = SteeringState()

    def reset(self) -> None:
        self.state = SteeringState()

    def step(self, command: float, dt: float) -> SteeringState:
        """
        command: desired centreline steer angle (rad), + left (CCW from above).
        Returns updated SteeringState with per-wheel angles.
        """
        cfg = self.cfg
        cmd = float(command)

        # Rate limit toward command
        max_step = abs(cfg.steering_rate) * max(dt, 1e-9)
        err = cmd - self.state.actual
        if abs(err) <= max_step:
            actual = cmd
        else:
            actual = self.state.actual + math.copysign(max_step, err)

        # Angle limit
        actual = max(-cfg.max_steer_angle, min(cfg.max_steer_angle, actual))

        # Ackermann → FL / FR
        if cfg.ackermann_enabled and abs(actual) > 1e-9:
            d_fl, d_fr = self._ackermann(actual)
        else:
            d_fl = d_fr = actual

        d_rl = d_rr = 0.0
        if cfg.rear_steer_enabled and abs(cfg.max_rear_steer) > 0:
            # reserved — not used in 14.9.1
            pass

        self.state = SteeringState(
            command=cmd,
            actual=actual,
            delta_fl=d_fl,
            delta_fr=d_fr,
            delta_rl=d_rl,
            delta_rr=d_rr,
        )
        return self.state

    def _ackermann(self, delta: float) -> tuple[float, float]:
        """
        Classical Ackermann for front axle.
        +delta = left turn → FL is inner, FR is outer.
        R = L / tan(δ)  (centreline), then
          δ_inner = atan(L / (R - T/2))
          δ_outer = atan(L / (R + T/2))
        """
        L = max(self.cfg.wheelbase, 0.5)
        T = max(self.cfg.track_front, 0.5)
        # Avoid singularity at δ→0 (handled by caller)
        tan_d = math.tan(delta)
        if abs(tan_d) < 1e-12:
            return delta, delta
        R = L / tan_d  # signed turning radius to centreline
        # Inner is the side toward the turn centre
        if delta > 0:  # left turn, R > 0, FL inner
            d_inner = math.atan(L / (R - T / 2))
            d_outer = math.atan(L / (R + T / 2))
            return d_inner, d_outer  # FL, FR
        else:  # right turn, R < 0, FR inner
            d_inner = math.atan(L / (R + T / 2))  # FR (more negative)
            d_outer = math.atan(L / (R - T / 2))  # FL
            return d_outer, d_inner  # FL, FR
