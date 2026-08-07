"""Plug-and-play subsystem registry."""
from __future__ import annotations

from typing import Any, Callable


class SubsystemRegistry:
    """Register factories for interchangeable modules."""

    def __init__(self) -> None:
        self._factories: dict[str, dict[str, Callable[..., Any]]] = {}

    def register(self, category: str, name: str, factory: Callable[..., Any]) -> None:
        self._factories.setdefault(category, {})[name] = factory

    def create(self, category: str, name: str, **kwargs: Any) -> Any:
        if category not in self._factories or name not in self._factories[category]:
            raise KeyError(f"Unknown subsystem {category}/{name}")
        return self._factories[category][name](**kwargs)

    def list(self, category: str | None = None) -> dict[str, list[str]]:
        if category:
            return {category: list(self._factories.get(category, {}).keys())}
        return {c: list(names.keys()) for c, names in self._factories.items()}

    def has(self, category: str, name: str) -> bool:
        return name in self._factories.get(category, {})


# Global default registry with known module names (factories are light stubs)
DEFAULT_REGISTRY = SubsystemRegistry()


def _register_defaults() -> None:
    DEFAULT_REGISTRY.register("tire", "dugoff", lambda **kw: {"model": "dugoff", **kw})
    DEFAULT_REGISTRY.register("tire", "pacejka", lambda **kw: {"model": "pacejka", **kw})
    DEFAULT_REGISTRY.register("powertrain", "ice", lambda **kw: {"architecture": "ice", **kw})
    DEFAULT_REGISTRY.register("powertrain", "parallel", lambda **kw: {"architecture": "parallel", **kw})
    DEFAULT_REGISTRY.register("powertrain", "ev", lambda **kw: {"architecture": "ev", **kw})
    DEFAULT_REGISTRY.register("differential", "open", lambda **kw: {"diff_type": "open", **kw})
    DEFAULT_REGISTRY.register("differential", "locked", lambda **kw: {"diff_type": "locked", **kw})
    DEFAULT_REGISTRY.register("differential", "clutch_lsd", lambda **kw: {"diff_type": "clutch_lsd", **kw})
    DEFAULT_REGISTRY.register("aero", "analytical", lambda **kw: {"mode": "analytical", **kw})
    DEFAULT_REGISTRY.register("aero", "lookup", lambda **kw: {"mode": "lookup", **kw})
    DEFAULT_REGISTRY.register("controls", "standard", lambda **kw: {"abs": True, "tc": True, "esc": True, **kw})
    DEFAULT_REGISTRY.register("driver", "pure_pursuit", lambda **kw: {"mode": "pure_pursuit", **kw})
    DEFAULT_REGISTRY.register("driver", "stanley", lambda **kw: {"mode": "stanley", **kw})


_register_defaults()
