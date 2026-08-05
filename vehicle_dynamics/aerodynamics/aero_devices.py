"""Configurable aerodynamic device bundle."""

from __future__ import annotations

from dataclasses import dataclass, field

from .wing_model import WingParams
from .diffuser_model import DiffuserParams, SplitterParams
from .drs import DRSParams
from .active_aero import ActiveAeroParams


@dataclass
class AeroDeviceConfig:
    """
    Master switch + per-device configuration.

    devices_enabled=False → ClosedLoopAero matches Phase 9.1 baseline.
    """

    devices_enabled: bool = True
    use_front_wing: bool = True
    use_rear_wing: bool = True
    use_diffuser: bool = True
    use_splitter: bool = True
    use_drs: bool = True
    use_active_aero: bool = True

    front_wing: WingParams = field(
        default_factory=lambda: WingParams(
            area=0.30, Cl0=0.85, Cl_alpha=3.2, Cd0=0.04, induced_factor=0.07
        )
    )
    rear_wing: WingParams = field(
        default_factory=lambda: WingParams(
            area=0.40, Cl0=1.1, Cl_alpha=3.8, Cd0=0.06, induced_factor=0.09
        )
    )
    diffuser: DiffuserParams = field(default_factory=DiffuserParams)
    splitter: SplitterParams = field(default_factory=SplitterParams)
    drs: DRSParams = field(default_factory=DRSParams)
    active: ActiveAeroParams = field(default_factory=ActiveAeroParams)

    # Fixed angles when active aero disabled
    front_wing_alpha: float = 0.10
    rear_wing_alpha: float = 0.12
