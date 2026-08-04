"""
Tire model factory — select Dugoff or Pacejka without API changes.
"""

from __future__ import annotations

from .dugoff import DugoffTire, DugoffParams
from .pacejka import PacejkaTire
from .pacejka_parameters import PacejkaParams, default_passenger_car, high_mu_race, low_mu_wet


def create_tire(model: str = "dugoff", **kwargs):
    """
    model: "dugoff" | "pacejka" | "pacejka_race" | "pacejka_wet"
    """
    key = model.lower().strip()
    if key in ("dugoff", "dugoff_standard", "standard_dugoff"):
        params = kwargs.get("params") or DugoffParams()
        return DugoffTire(params)
    if key in ("pacejka", "pacejka_default", "mf"):
        params = kwargs.get("params") or default_passenger_car()
        return PacejkaTire(params)
    if key in ("pacejka_race", "race"):
        return PacejkaTire(kwargs.get("params") or high_mu_race())
    if key in ("pacejka_wet", "wet"):
        return PacejkaTire(kwargs.get("params") or low_mu_wet())
    raise ValueError(f"Unknown tire model: {model!r}")
