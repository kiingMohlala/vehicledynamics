"""Numerical stability monitoring."""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np

from vehicle_dynamics.simulation.telemetry_recorder import TelemetryRecorder


@dataclass
class NumericalReport:
    has_nan: bool = False
    has_inf: bool = False
    max_abs_vx: float = 0.0
    max_abs_ax: float = 0.0
    max_abs_ay: float = 0.0
    max_slip: float = 0.0
    timestep_stable: bool = True
    messages: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return (not self.has_nan) and (not self.has_inf) and self.timestep_stable


class NumericalMonitor:
    VX_LIMIT = 120.0      # m/s ~ 430 km/h sanity
    AX_LIMIT = 30.0       # m/s²
    AY_LIMIT = 30.0
    SLIP_LIMIT = 1.5

    def check(self, log: TelemetryRecorder, dt: float) -> NumericalReport:
        rep = NumericalReport()
        if not log.samples:
            rep.messages.append("empty telemetry")
            return rep
        d = log.to_numpy()
        for key, arr in d.items():
            if not np.issubdtype(arr.dtype, np.number):
                continue
            if np.any(np.isnan(arr)):
                rep.has_nan = True
                rep.messages.append(f"NaN in {key}")
            if np.any(np.isinf(arr)):
                rep.has_inf = True
                rep.messages.append(f"Inf in {key}")
        rep.max_abs_vx = float(np.max(np.abs(d.get("vx", [0]))))
        rep.max_abs_ax = float(np.max(np.abs(d.get("ax", [0]))))
        rep.max_abs_ay = float(np.max(np.abs(d.get("ay", [0]))))
        rep.max_slip = float(np.max(np.abs(d.get("slip_max", [0]))))
        if rep.max_abs_vx > self.VX_LIMIT:
            rep.messages.append(f"vx sanity {rep.max_abs_vx}")
            rep.timestep_stable = False
        if rep.max_abs_ax > self.AX_LIMIT or rep.max_abs_ay > self.AY_LIMIT:
            rep.messages.append("accel sanity exceeded")
            rep.timestep_stable = False
        if rep.max_slip > self.SLIP_LIMIT:
            rep.messages.append("slip sanity exceeded")
        # Monotonic time
        t = d["time"]
        if len(t) > 1 and np.any(np.diff(t) <= 0):
            rep.timestep_stable = False
            rep.messages.append("non-monotonic time")
        if len(t) > 1:
            dt_obs = float(np.median(np.diff(t)))
            if abs(dt_obs - dt) > dt * 0.5:
                rep.messages.append(f"dt drift {dt_obs} vs {dt}")
        return rep
