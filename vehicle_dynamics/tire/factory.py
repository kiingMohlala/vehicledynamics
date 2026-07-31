from .dugoff import DugoffTire, DugoffParams
from .base import TireModel

class TireFactory:
    @staticmethod
    def create(name: str = "dugoff_standard", params: DugoffParams = None) -> TireModel:
        """
        Create a tire model instance.

        Parameters
        ----------
        name : str
            Model identifier ("dugoff", "dugoff_standard", "standard")
        params : DugoffParams, optional
            Custom tire parameters. If None, defaults are used.
        """
        if name in ("dugoff", "dugoff_standard", "standard"):
            return DugoffTire(params if params is not None else DugoffParams())
        raise ValueError(
            f"Unknown tire model: {name}. Available: dugoff / dugoff_standard / standard"
        )
