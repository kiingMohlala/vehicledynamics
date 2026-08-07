"""Support definitions for structural models."""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class Support:
    node_tag: str
    fix_ux: bool = True
    fix_uy: bool = True
    fix_uz: bool = True
    fix_rx: bool = True
    fix_ry: bool = True
    fix_rz: bool = True


def simply_supported_beam() -> list[Support]:
    return [
        Support("A", fix_rx=False, fix_ry=False, fix_rz=False),
        Support("B", fix_ux=False, fix_rx=False, fix_ry=False, fix_rz=False),
    ]


def fixed_cantilever() -> list[Support]:
    return [Support("root")]
