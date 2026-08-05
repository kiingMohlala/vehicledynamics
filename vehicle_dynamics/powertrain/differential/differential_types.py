"""Differential type enum."""

from __future__ import annotations

from enum import Enum


class DiffType(str, Enum):
    OPEN = "open"
    LOCKED = "locked"
    CLUTCH_LSD = "clutch_lsd"
    VISCOUS = "viscous"
    TORSEN = "torsen"
    TORQUE_VECTORING = "torque_vectoring"
