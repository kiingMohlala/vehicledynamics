"""Generic CSV import utilities."""
from __future__ import annotations

from pathlib import Path
from typing import Any
import numpy as np


def load_csv_columns(path: str | Path, delimiter: str = ",") -> dict[str, np.ndarray]:
    path = Path(path)
    with open(path) as f:
        header = f.readline().strip().split(delimiter)
    data = np.genfromtxt(path, delimiter=delimiter, skip_header=1)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    out = {}
    for i, name in enumerate(header):
        if i < data.shape[1]:
            out[name.strip()] = data[:, i]
    return out


def load_xy_path(path: str | Path) -> tuple[np.ndarray, np.ndarray]:
    cols = load_csv_columns(path)
    # flexible column names
    for xk, yk in (("x", "y"), ("X", "Y"), ("lon", "lat")):
        if xk in cols and yk in cols:
            return cols[xk], cols[yk]
    keys = list(cols.keys())
    return cols[keys[0]], cols[keys[1]]
