"""Freeze complete simulation configurations for reproducibility."""
from __future__ import annotations

from dataclasses import dataclass, field, asdict
from typing import Any
import hashlib
import json
import time


@dataclass
class ConfigurationSnapshot:
    config_id: str
    vehicle: dict[str, Any]
    track: dict[str, Any] = field(default_factory=dict)
    weather: dict[str, Any] = field(default_factory=dict)
    controllers: dict[str, Any] = field(default_factory=dict)
    tire_model: str = ""
    solver: dict[str, Any] = field(default_factory=dict)
    software_version: str = ""
    timestamp: float = field(default_factory=time.time)

    def config_hash(self) -> str:
        payload = json.dumps(asdict(self), sort_keys=True, default=str)
        return hashlib.sha256(payload.encode()).hexdigest()[:16]


@dataclass
class ConfigurationControl:
    snapshots: list[ConfigurationSnapshot] = field(default_factory=list)

    def freeze(self, snap: ConfigurationSnapshot) -> str:
        self.snapshots.append(snap)
        return snap.config_hash()

    def get(self, config_id: str) -> ConfigurationSnapshot:
        for s in self.snapshots:
            if s.config_id == config_id:
                return s
        raise KeyError(config_id)
