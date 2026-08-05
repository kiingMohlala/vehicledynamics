from abc import ABC, abstractmethod
from .dugoff import TireState


class TireModel(ABC):
    @abstractmethod
    def longitudinal_lateral_force(
        self,
        slip_ratio: float,
        slip_angle: float,
        normal_load: float,
        camber_rad: float = 0.0,
    ) -> TireState:
        ...
