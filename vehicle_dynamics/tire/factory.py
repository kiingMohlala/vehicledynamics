from .dugoff import DugoffTire, DugoffParams
from .base import TireModel

class TireFactory:
    @staticmethod
    def create(name: str, **kwargs) -> TireModel:
        if name in ("dugoff", "dugoff_standard", "standard"):
            return DugoffTire(DugoffParams(**kwargs) if kwargs else DugoffParams())
        else:
            raise ValueError(f"Unknown tire model: {name}. Available: dugoff / dugoff_standard")
