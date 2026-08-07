"""Coordinate frame transforms (ISO vehicle, SAE, ENU, NED)."""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class Frame:
    name: str  # iso | sae | enu | ned


# ISO 8855 vehicle: x forward, y left, z up
# SAE J670: x forward, y right, z down
# ENU: East, North, Up (world)
# NED: North, East, Down (world)


def iso_to_sae(v_iso: np.ndarray) -> np.ndarray:
    """[x,y,z]_ISO → [x,y,z]_SAE."""
    x, y, z = v_iso
    return np.array([x, -y, -z], dtype=float)


def sae_to_iso(v_sae: np.ndarray) -> np.ndarray:
    x, y, z = v_sae
    return np.array([x, -y, -z], dtype=float)


def enu_to_ned(v_enu: np.ndarray) -> np.ndarray:
    e, n, u = v_enu
    return np.array([n, e, -u], dtype=float)


def ned_to_enu(v_ned: np.ndarray) -> np.ndarray:
    n, e, d = v_ned
    return np.array([e, n, -d], dtype=float)


def rotate_yaw(vec: np.ndarray, yaw_rad: float) -> np.ndarray:
    c, s = np.cos(yaw_rad), np.sin(yaw_rad)
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)
    return R @ np.asarray(vec, dtype=float)


def transform(vec: np.ndarray, from_frame: str, to_frame: str, yaw_rad: float = 0.0) -> np.ndarray:
    v = np.asarray(vec, dtype=float).reshape(3)
    src, dst = from_frame.lower(), to_frame.lower()
    if src == dst:
        return v.copy()
    # Normalize via ISO
    if src == "sae":
        v = sae_to_iso(v)
    elif src == "ned":
        v = ned_to_enu(v)  # then treat as world-like; for vehicle use yaw
    elif src == "enu":
        pass
    elif src != "iso":
        raise KeyError(src)
    if src in ("enu", "ned") or dst in ("enu", "ned"):
        v = rotate_yaw(v, yaw_rad)
    if dst == "sae":
        v = iso_to_sae(v)
    elif dst == "ned":
        v = enu_to_ned(v)
    elif dst == "iso":
        pass
    elif dst == "enu":
        pass
    else:
        raise KeyError(dst)
    return v
