"""Structured handling report."""

from __future__ import annotations

from dataclasses import dataclass, field
from .metrics import SteadyStateMetrics, UtilizationMetrics, StabilityMetrics, DriverMetrics
from .balance import BalanceResult


@dataclass
class HandlingReport:
    steady: SteadyStateMetrics
    utilization: UtilizationMetrics
    balance: BalanceResult
    stability: StabilityMetrics
    driver: DriverMetrics
    warnings: list[str] = field(default_factory=list)

    def format_text(self) -> str:
        s = self.steady
        b = self.balance
        st = self.stability
        d = self.driver
        u = self.utilization
        lines = [
            "Handling Report",
            "",
            "Steady-state",
            "--------------",
            f"Understeer gradient : {s.understeer_gradient_deg_per_g:+.2f} deg/g",
            f"Yaw gain            : {s.yaw_rate_gain:.3f} (r/ay)",
            f"Steering gain       : {s.steering_gain:.2f} (ay/delta)",
            f"Max lateral accel   : {s.max_ay_g:.2f} g",
            f"Turning radius      : {s.turning_radius:.1f} m",
        ]
        if s.characteristic_speed is not None:
            lines.append(f"Characteristic speed: {s.characteristic_speed:.1f} m/s")
        if s.critical_speed is not None:
            lines.append(f"Critical speed      : {s.critical_speed:.1f} m/s")
        lines += [
            "",
            "Balance",
            "--------------",
            f"Classification      : {b.classification}",
            f"Front utilization   : {b.front_utilization:.2f}",
            f"Rear utilization    : {b.rear_utilization:.2f}",
            f"Limiting axle       : {u.limiting_axle}",
            f"Limiting wheel      : {u.limiting_wheel}",
        ]
        if b.notes:
            lines.append(f"Notes               : {b.notes}")
        lines += [
            "",
            "Stability",
            "--------------",
            f"Peak beta           : {st.peak_beta_deg:.1f}°",
            f"RMS beta            : {st.rms_beta_deg:.1f}°",
            f"Peak yaw            : {st.peak_yaw_rate:.2f} rad/s",
            f"RMS yaw             : {st.rms_yaw_rate:.2f} rad/s",
        ]
        if st.peak_load_transfer is not None:
            lines.append(f"Peak load transfer  : {st.peak_load_transfer:.0f} N")
        if st.peak_jacking is not None:
            lines.append(f"Peak jacking        : {st.peak_jacking:.0f} N")
        if st.peak_rc_migration is not None:
            lines.append(f"Peak RC migration   : {st.peak_rc_migration:.3f} m")
        lines += [
            "",
            "Driver / path",
            "--------------",
            f"Max steering        : {d.max_steer_deg:.1f}°",
            f"Entry speed         : {d.entry_speed:.1f} m/s",
            f"Exit speed          : {d.exit_speed:.1f} m/s",
            f"Average speed       : {d.average_speed:.1f} m/s",
            f"Duration            : {d.corner_time:.2f} s",
        ]
        if d.stopping_distance is not None:
            lines.append(f"Stopping distance   : {d.stopping_distance:.1f} m")
        if d.stop_100_0_kmh is not None:
            lines.append(f"100–0 km/h equiv.   : {d.stop_100_0_kmh:.1f} m")
        if self.warnings:
            lines += ["", "Warnings", "--------------"]
            for w in self.warnings:
                lines.append(f"- {w}")
        return "\n".join(lines)
