from dataclasses import dataclass, field
from ..lateral.parameters import BicycleParameters
from ..lateral.load_transfer import LoadTransferParameters
from ..braking.parameters import VehicleLongitudinalParams, BrakeParams
from ..tire.dugoff import DugoffParams
from .steering import SteeringParameters


@dataclass
class DualTrackParameters:
    bicycle: BicycleParameters = field(default_factory=BicycleParameters)
    load_transfer: LoadTransferParameters = field(default_factory=LoadTransferParameters)
    longitudinal: VehicleLongitudinalParams = field(default_factory=VehicleLongitudinalParams)
    brake: BrakeParams = field(default_factory=BrakeParams)
    tire: DugoffParams = field(default_factory=DugoffParams)
    steering: SteeringParameters = field(default_factory=SteeringParameters)

    def __post_init__(self):
        self.longitudinal.mass = self.bicycle.m
