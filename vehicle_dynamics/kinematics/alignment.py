"""Alignment angles from upright / hardpoint geometry."""
from __future__ import annotations

import numpy as np


def camber_from_upright(wheel_center: np.ndarray, contact: np.ndarray) -> float:
    """Camber (rad): tilt of wheel plane in YZ. Negative = top inward."""
    v = contact - wheel_center
    # angle from vertical
    return float(np.arctan2(v[1], -v[2] if abs(v[2]) > 1e-12 else -1e-12))


def toe_from_heading(wheel_heading_xy: np.ndarray, vehicle_x: np.ndarray = np.array([1.0, 0.0])) -> float:
    """Toe (rad): yaw of wheel heading vs vehicle X. Positive = toe-in for left? use signed."""
    h = wheel_heading_xy / (np.linalg.norm(wheel_heading_xy) + 1e-15)
    return float(np.arctan2(h[1], h[0]))


def caster_from_kingpin(upper: np.ndarray, lower: np.ndarray) -> float:
    """Caster (rad): kingpin axis lean in XZ (positive = upper rearward)."""
    v = upper - lower
    return float(np.arctan2(-v[0], v[2] if abs(v[2]) > 1e-12 else 1e-12))


def kpi_from_kingpin(upper: np.ndarray, lower: np.ndarray) -> float:
    """Kingpin inclination (rad) in YZ."""
    v = upper - lower
    return float(np.arctan2(abs(v[1]), abs(v[2]) if abs(v[2]) > 1e-12 else 1e-12))


def scrub_radius(wheel_center: np.ndarray, contact: np.ndarray, upper: np.ndarray, lower: np.ndarray) -> float:
    """
    Lateral distance at ground between kingpin axis intersection and contact center.
    """
    # kingpin direction
    d = upper - lower
    d = d / (np.linalg.norm(d) + 1e-15)
    # intersect axis with ground z = contact_z
    z_g = contact[2]
    if abs(d[2]) < 1e-12:
        axis_at_ground = lower.copy()
        axis_at_ground[2] = z_g
    else:
        t = (z_g - lower[2]) / d[2]
        axis_at_ground = lower + t * d
    return float(contact[1] - axis_at_ground[1])


def mechanical_trail(wheel_center: np.ndarray, contact: np.ndarray, upper: np.ndarray, lower: np.ndarray) -> float:
    """Longitudinal offset at ground between kingpin axis and contact."""
    d = upper - lower
    d = d / (np.linalg.norm(d) + 1e-15)
    z_g = contact[2]
    if abs(d[2]) < 1e-12:
        axis_at_ground = lower.copy()
        axis_at_ground[2] = z_g
    else:
        t = (z_g - lower[2]) / d[2]
        axis_at_ground = lower + t * d
    return float(axis_at_ground[0] - contact[0])
