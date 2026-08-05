"""Fixed subsystem update order."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class UpdateScheduler:
    """
    Deterministic order:
    1 driver → 2 controls → 3 powertrain → 4 differential
    → 5 brakes → 6 aero → 7 vehicle dynamics → 8 telemetry
    """

    order: list[str] = field(
        default_factory=lambda: [
            "events",
            "driver",
            "controls",
            "powertrain",
            "differential",
            "brakes",
            "aero",
            "vehicle",
            "telemetry",
        ]
    )

    def validate(self) -> bool:
        required = {"driver", "controls", "vehicle", "telemetry"}
        return required.issubset(set(self.order))
