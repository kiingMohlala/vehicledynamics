from dataclasses import dataclass, field
from ..lateral.parameters import BicycleParameters
from ..braking.parameters import VehicleLongitudinalParams, BrakeParams
from ..tire.dugoff import DugoffParams

@dataclass
class CombinedParameters:
    """Composition of lateral + longitudinal parameters for Phase 4.2."""
    bicycle: BicycleParameters = field(default_factory=BicycleParameters)
    longitudinal: VehicleLongitudinalParams = field(default_factory=VehicleLongitudinalParams)
    brake: BrakeParams = field(default_factory=BrakeParams)
    tire: DugoffParams = field(default_factory=DugoffParams)
    # Keep masses consistent by default
    def __post_init__(self):
        # Prefer bicycle mass as the planar-model mass
        self.longitudinal.mass = self.bicycle.m
