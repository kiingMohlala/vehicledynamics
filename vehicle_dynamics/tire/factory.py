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
            "dugoff" / "dugoff_standard" / "standard" — with camber support
            "dugoff_no_camber" — camber_enabled=False (Phase 3.4 equivalent)
        params : DugoffParams, optional
        """
        if name in ("dugoff", "dugoff_standard", "standard"):
            p = params if params is not None else DugoffParams()
            return DugoffTire(p)
        if name in ("dugoff_no_camber", "phase34"):
            p = params if params is not None else DugoffParams()
            p.camber_enabled = False
            return DugoffTire(p)
        raise ValueError(
            f"Unknown tire model: {name}. "
            "Available: dugoff / dugoff_standard / dugoff_no_camber"
        )
