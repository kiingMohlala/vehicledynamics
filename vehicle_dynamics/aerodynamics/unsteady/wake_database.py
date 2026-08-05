"""Simple registry of nearby vehicles for wake evaluation."""

from __future__ import annotations

from dataclasses import dataclass, field
from .wake_model import WakeSource, WakeField
from .drafting import DraftingParams


@dataclass
class WakeDatabase:
    field: WakeField = field(default_factory=WakeField)

    def clear(self) -> None:
        self.field.sources.clear()

    def set_sources(self, sources: list[WakeSource]) -> None:
        self.field.sources = list(sources)

    def add_vehicle(self, x: float, y: float = 0.0, heading: float = 0.0, strength: float = 1.0) -> None:
        self.field.sources.append(WakeSource(x=x, y=y, heading=heading, strength=strength))

    def set_params(self, params: DraftingParams) -> None:
        self.field.params = params
