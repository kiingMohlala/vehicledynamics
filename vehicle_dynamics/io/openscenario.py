"""
Minimal OpenSCENARIO (.xosc) / JSON scenario loader.

Supports:
  - named scenario metadata
  - weather parameters
  - event list (time, action)
  - maneuver hints (double_lane_change, emergency_brake, ...)
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from xml.etree import ElementTree as ET
from typing import Any
import json


@dataclass
class ScenarioEvent:
    time: float
    name: str
    action: str
    params: dict[str, Any] = field(default_factory=dict)


@dataclass
class OpenScenario:
    name: str
    weather: dict[str, Any] = field(default_factory=dict)
    events: list[ScenarioEvent] = field(default_factory=list)
    maneuvers: list[str] = field(default_factory=list)
    metadata: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "weather": self.weather,
            "maneuvers": self.maneuvers,
            "events": [
                {"time": e.time, "name": e.name, "action": e.action, "params": e.params}
                for e in self.events
            ],
            "metadata": self.metadata,
        }


def _local(tag: str) -> str:
    return tag.split("}", 1)[1] if "}" in tag else tag


def load_openscenario(path: str | Path) -> OpenScenario:
    path = Path(path)
    if path.suffix.lower() == ".json":
        data = json.loads(path.read_text())
        events = [
            ScenarioEvent(
                time=float(e.get("time", 0)),
                name=str(e.get("name", "")),
                action=str(e.get("action", "")),
                params=dict(e.get("params", {})),
            )
            for e in data.get("events", [])
        ]
        return OpenScenario(
            name=data.get("name", path.stem),
            weather=data.get("weather", {}),
            events=events,
            maneuvers=list(data.get("maneuvers", [])),
            metadata=data.get("metadata", {}),
        )

    # XML .xosc (minimal)
    tree = ET.parse(path)
    root = tree.getroot()
    name = path.stem
    weather: dict[str, Any] = {}
    events: list[ScenarioEvent] = []
    maneuvers: list[str] = []

    for elem in root.iter():
        t = _local(elem.tag)
        if t == "Scenario" or t == "OpenSCENARIO":
            name = elem.attrib.get("name", name)
        if t in ("Weather", "Environment"):
            weather.update({k: v for k, v in elem.attrib.items()})
        if t == "Event":
            events.append(
                ScenarioEvent(
                    time=float(elem.attrib.get("time", elem.attrib.get("priority", 0) or 0)),
                    name=elem.attrib.get("name", "event"),
                    action=elem.attrib.get("action", "trigger"),
                )
            )
        if t == "Maneuver":
            maneuvers.append(elem.attrib.get("name", "maneuver"))

    return OpenScenario(name=name, weather=weather, events=events, maneuvers=maneuvers)


def write_minimal_openscenario(path: str | Path, name: str = "double_lane_change") -> Path:
    path = Path(path)
    if path.suffix.lower() == ".json":
        data = {
            "name": name,
            "weather": {"condition": "dry", "visibility": 10000},
            "maneuvers": [name],
            "events": [
                {"time": 2.0, "name": "start_swerve", "action": "steer", "params": {"angle_deg": 5}},
                {"time": 4.0, "name": "return", "action": "steer", "params": {"angle_deg": -5}},
            ],
        }
        path.write_text(json.dumps(data, indent=2))
    else:
        path.write_text(
            f'''<?xml version="1.0"?>
<OpenSCENARIO>
  <Scenario name="{name}"/>
  <Weather condition="dry"/>
  <Maneuver name="{name}"/>
  <Event name="start" time="2.0" action="steer"/>
</OpenSCENARIO>
'''
        )
    return path
