"""Powertrain state report."""

from __future__ import annotations

from .powertrain_solver import PowertrainState


def format_powertrain_report(state: PowertrainState, title: str = "Powertrain") -> str:
    e = state.engine
    lines = [
        f"=== {title} ===",
        f"Time:          {state.time:.2f} s",
        f"RPM:           {e.rpm:.0f}",
        f"Throttle:      {e.throttle*100:.1f} %",
        f"Torque out:    {e.torque_output:.1f} N·m",
        f"Torque ind:    {e.torque_indicated:.1f} N·m",
        f"Engine brake:  {e.torque_brake:.1f} N·m",
        f"Power:         {e.power_kw:.1f} kW",
        f"Limiter:       {e.limiter_factor:.2f}",
        f"Load torque:   {state.load_torque:.1f} N·m",
        f"Fuel rate:     {state.fuel.fuel_rate_gps:.2f} g/s",
        f"Fuel total:    {state.fuel.fuel_total_g:.1f} g",
        f"BSFC:          {state.fuel.bsfc_gpkwh:.0f} g/kWh",
        f"Coolant:       {state.thermal.coolant_C:.1f} °C",
        f"Oil:           {state.thermal.oil_C:.1f} °C",
        f"Therm. eff:    {state.thermal.efficiency_factor:.2f}",
    ]
    return "\n".join(lines)
