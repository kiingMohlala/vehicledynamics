"""Scheduled scenario events."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Any


@dataclass
class SimulationEvent:
    time: float
    name: str
    action: Callable[[Any], None]
    fired: bool = False


@dataclass
class EventManager:
    events: list[SimulationEvent] = field(default_factory=list)

    def add(self, time: float, name: str, action: Callable[[Any], None]) -> None:
        self.events.append(SimulationEvent(time=time, name=name, action=action))
        self.events.sort(key=lambda e: e.time)

    def clear(self) -> None:
        self.events.clear()

    def process(self, time: float, sim: Any) -> list[str]:
        fired = []
        for ev in self.events:
            if not ev.fired and time >= ev.time:
                ev.action(sim)
                ev.fired = True
                fired.append(ev.name)
        return fired
