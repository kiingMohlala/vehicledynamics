"""JSON / YAML serialization for vehicle definitions."""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from .vehicle_definition import VehicleDefinition


def save_json(defn: VehicleDefinition, path: str | Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        json.dump(defn.to_dict(), f, indent=2)


def load_json(path: str | Path) -> VehicleDefinition:
    with open(path) as f:
        return VehicleDefinition.from_dict(json.load(f))


def save_yaml(defn: VehicleDefinition, path: str | Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    data = defn.to_dict()
    try:
        import yaml  # type: ignore
        with open(path, "w") as f:
            yaml.safe_dump(data, f, sort_keys=False)
    except ImportError:
        # Minimal YAML-ish dump without PyYAML
        with open(path, "w") as f:
            f.write(_simple_yaml(data))


def load_yaml(path: str | Path) -> VehicleDefinition:
    path = Path(path)
    try:
        import yaml  # type: ignore
        with open(path) as f:
            data = yaml.safe_load(f)
        return VehicleDefinition.from_dict(data)
    except ImportError:
        # Fallback: if file is JSON-compatible YAML (our simple dump), try JSON
        text = path.read_text()
        # Our simple writer embeds JSON under a marker for round-trip without PyYAML
        if text.startswith("#VD_JSON\n"):
            return VehicleDefinition.from_dict(json.loads(text.split("\n", 1)[1]))
        raise RuntimeError("PyYAML not installed and file is not VD_JSON fallback format")


def _simple_yaml(data: dict[str, Any]) -> str:
    """Fallback that embeds JSON for reliable round-trip without PyYAML."""
    return "#VD_JSON\n" + json.dumps(data, indent=2)


def roundtrip_json(defn: VehicleDefinition) -> VehicleDefinition:
    raw = json.dumps(defn.to_dict())
    return VehicleDefinition.from_dict(json.loads(raw))
