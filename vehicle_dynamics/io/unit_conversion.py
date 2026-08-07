"""Unit conversion helpers."""
from __future__ import annotations

import math

# Length
def m_to_ft(m: float) -> float: return m * 3.280839895
def ft_to_m(ft: float) -> float: return ft / 3.280839895
def m_to_mi(m: float) -> float: return m / 1609.344
def kmh_to_ms(v: float) -> float: return v / 3.6
def ms_to_kmh(v: float) -> float: return v * 3.6
def mph_to_ms(v: float) -> float: return v * 0.44704
def ms_to_mph(v: float) -> float: return v / 0.44704

# Angle
def deg_to_rad(d: float) -> float: return d * math.pi / 180.0
def rad_to_deg(r: float) -> float: return r * 180.0 / math.pi

# Force / torque / power
def N_to_lbf(n: float) -> float: return n * 0.224809
def Nm_to_ftlb(t: float) -> float: return t * 0.737562
def kW_to_hp(p: float) -> float: return p * 1.341022

# Mass
def kg_to_lb(m: float) -> float: return m * 2.2046226

def convert(value: float, from_unit: str, to_unit: str) -> float:
    key = (from_unit.lower(), to_unit.lower())
    table = {
        ("m", "ft"): m_to_ft, ("ft", "m"): ft_to_m,
        ("m/s", "km/h"): ms_to_kmh, ("km/h", "m/s"): kmh_to_ms,
        ("m/s", "mph"): ms_to_mph, ("mph", "m/s"): mph_to_ms,
        ("deg", "rad"): deg_to_rad, ("rad", "deg"): rad_to_deg,
        ("kw", "hp"): kW_to_hp, ("kg", "lb"): kg_to_lb,
        ("n", "lbf"): N_to_lbf, ("nm", "ft-lb"): Nm_to_ftlb,
    }
    if key not in table:
        if from_unit.lower() == to_unit.lower():
            return value
        raise KeyError(f"No conversion {from_unit} → {to_unit}")
    return table[key](value)
