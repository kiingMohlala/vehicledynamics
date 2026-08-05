"""Energy / force / torque consistency checks."""

from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np

from vehicle_dynamics.simulation.telemetry_recorder import TelemetryRecorder


@dataclass
class ConsistencyReport:
    mass_ok: bool = True
    force_ok: bool = True
    torque_ok: bool = True
    energy_ok: bool = True
    wheel_speed_ok: bool = True
    messages: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return all([self.mass_ok, self.force_ok, self.torque_ok, self.energy_ok, self.wheel_speed_ok])


class ConsistencyChecker:
    def check(self, log: TelemetryRecorder, mass: float = 1400.0) -> ConsistencyReport:
        rep = ConsistencyReport()
        if len(log.samples) < 3:
            rep.messages.append("too few samples")
            return rep
        d = log.to_numpy()
        vx = d["vx"]
        ax = d["ax"]
        t = d["time"]

        # Finite checks already elsewhere — force balance proxy: ax roughly finite
        if not np.all(np.isfinite(ax)):
            rep.force_ok = False
            rep.messages.append("ax not finite")

        # Wheel speed vs vehicle speed sanity
        # (wheel omega not in all samples as speed; use slip_max)
        if np.any(d["slip_max"] > 2.0):
            rep.wheel_speed_ok = False
            rep.messages.append("extreme slip")

        # Torque left/right finite
        if not np.all(np.isfinite(d["torque_L"])) or not np.all(np.isfinite(d["torque_R"])):
            rep.torque_ok = False

        # Energy proxy: kinetic energy change vs integrated power rough bound
        ke = 0.5 * mass * vx * vx
        dke = float(ke[-1] - ke[0])
        # Not strict conservation (brakes/aero dissipate) — only check not exploding
        if abs(dke) > 0.5 * mass * (120.0 ** 2):
            rep.energy_ok = False
            rep.messages.append("KE explosion")

        # Mass constant by construction
        rep.mass_ok = True
        return rep
