"""
Phase 5.5 – Scenario definitions for system integration benchmark.

Each scenario returns (name, kwargs for FixedStepDualTrack.simulate).
"""

from __future__ import annotations

import numpy as np


def _const(v):
    return lambda t: float(v)


def _step(t0, value, t_end=None):
    def f(t):
        if t < t0:
            return 0.0
        if t_end is not None and t > t_end:
            return 0.0
        return float(value)
    return f


def scenario_catalog():
    """Return list of (name, sim_kwargs, flags)."""
    deg = np.deg2rad
    scenarios = []

    # --- Steering ---
    scenarios.append((
        "step_steer",
        dict(vx0=20.0, t_span=(0, 6),
             delta_func=_step(0.5, deg(6)),
             pedal_func=_const(0), throttle_func=_const(0)),
        dict(enable_esc=False, enable_tv=False, use_abs=False),
    ))
    scenarios.append((
        "sine_steer",
        dict(vx0=18.0, t_span=(0, 8),
             delta_func=lambda t: deg(5) * np.sin(2 * np.pi * 0.4 * max(t - 0.5, 0)),
             pedal_func=_const(0), throttle_func=_const(0)),
        dict(enable_esc=False, enable_tv=False, use_abs=False),
    ))
    scenarios.append((
        "double_lane_change",
        dict(vx0=20.0, t_span=(0, 8),
             delta_func=lambda t: (
                 deg(4) if 0.8 <= t < 1.6 else
                 deg(-4) if 2.4 <= t < 3.2 else
                 deg(-4) if 4.0 <= t < 4.8 else
                 deg(4) if 5.6 <= t < 6.4 else 0.0
             ),
             pedal_func=_const(0), throttle_func=_const(0)),
        dict(enable_esc=True, enable_tv=False, use_abs=False),
    ))

    # --- Braking ---
    scenarios.append((
        "emergency_brake",
        dict(vx0=25.0, t_span=(0, 6),
             delta_func=_const(0),
             pedal_func=_step(0.5, 1.0),
             throttle_func=_const(0)),
        dict(enable_esc=False, enable_tv=False, use_abs=True),
    ))
    scenarios.append((
        "split_mu_brake",
        dict(vx0=20.0, t_span=(0, 6),
             delta_func=_const(0),
             pedal_func=_step(0.5, 0.9),
             throttle_func=_const(0)),
        dict(enable_esc=True, enable_tv=False, use_abs=True,
             mu_wheels=np.array([0.4, 1.0, 0.4, 1.0])),
    ))
    scenarios.append((
        "trail_brake",
        dict(vx0=22.0, t_span=(0, 7),
             delta_func=_step(0.8, deg(5)),
             pedal_func=_step(1.2, 0.5, t_end=4.0),
             throttle_func=_const(0)),
        dict(enable_esc=True, enable_tv=False, use_abs=True),
    ))

    # --- Acceleration ---
    scenarios.append((
        "straight_accel",
        dict(vx0=5.0, t_span=(0, 5),
             delta_func=_const(0),
             pedal_func=_const(0),
             throttle_func=_step(0.3, 0.8)),
        dict(enable_esc=False, enable_tv=True, use_abs=False),
    ))
    scenarios.append((
        "corner_exit",
        dict(vx0=12.0, t_span=(0, 6),
             delta_func=_step(0.4, deg(5)),
             pedal_func=_const(0),
             throttle_func=_step(0.8, 0.6)),
        dict(enable_esc=False, enable_tv=True, use_abs=False),
    ))
    scenarios.append((
        "split_mu_launch",
        dict(vx0=5.0, t_span=(0, 5),
             delta_func=_const(0),
             pedal_func=_const(0),
             throttle_func=_step(0.3, 0.4)),
        dict(enable_esc=True, enable_tv=True, use_abs=False,
             mu_wheels=np.array([0.4, 1.0, 0.4, 1.0])),
    ))

    # --- Combined ---
    scenarios.append((
        "power_on_oversteer",
        dict(vx0=15.0, t_span=(0, 6),
             delta_func=_step(0.5, deg(7)),
             pedal_func=_const(0),
             throttle_func=_step(1.0, 0.7)),
        dict(enable_esc=True, enable_tv=True, use_abs=False),
    ))
    scenarios.append((
        "lift_off_oversteer",
        dict(vx0=20.0, t_span=(0, 6),
             delta_func=_step(0.5, deg(6)),
             pedal_func=_const(0),
             throttle_func=_step(0.0, 0.5, t_end=1.5)),  # throttle cut after 1.5
        dict(enable_esc=True, enable_tv=False, use_abs=False),
    ))
    scenarios.append((
        "abs_esc_tv_combined",
        dict(vx0=22.0, t_span=(0, 7),
             delta_func=_step(0.6, deg(6)),
             pedal_func=_step(1.5, 0.6),
             throttle_func=_step(0.3, 0.4, t_end=1.2)),
        dict(enable_esc=True, enable_tv=True, use_abs=True),
    ))

    return scenarios
