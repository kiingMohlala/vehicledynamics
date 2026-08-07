"""Maps drive mode → hybrid energy mode string for Phase 10.4."""

from __future__ import annotations

from dataclasses import dataclass

from .drive_modes import DriveMode


@dataclass
class HybridStrategy:
    def energy_mode_for(self, drive_mode: DriveMode | str, soc: float = 0.5) -> str:
        m = DriveMode(drive_mode) if isinstance(drive_mode, str) else drive_mode
        if m == DriveMode.ECO:
            return "economy" if soc > 0.2 else "charge_sustain"
        if m in (DriveMode.SPORT, DriveMode.TRACK, DriveMode.DRAG):
            return "performance"
        if m in (DriveMode.WET, DriveMode.SNOW):
            return "hybrid"
        if m == DriveMode.NORMAL:
            return "charge_deplete" if soc > 0.4 else "hybrid"
        return "hybrid"
