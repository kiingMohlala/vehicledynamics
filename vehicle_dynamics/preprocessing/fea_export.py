"""FEA exports: CalculiX/Abaqus INP, Code_Aster mesh stub, VTK."""
from __future__ import annotations

from pathlib import Path
from typing import Any
import numpy as np


def export_calculix(
    vertices: np.ndarray,
    faces: np.ndarray,
    path: str | Path,
    material_name: str = "Steel",
    E: float = 210e9,
    nu: float = 0.3,
) -> Path:
    """Shell triangle mesh as CalculiX .inp."""
    path = Path(path)
    lines = ["*HEADING", "vehicle_dynamics FEA export", "*NODE"]
    for i, v in enumerate(vertices, start=1):
        lines.append(f"{i}, {v[0]:.8e}, {v[1]:.8e}, {v[2]:.8e}")
    lines.append("*ELEMENT, TYPE=S3, ELSET=Body")
    for i, f in enumerate(faces, start=1):
        lines.append(f"{i}, {f[0]+1}, {f[1]+1}, {f[2]+1}")
    lines += [
        f"*MATERIAL, NAME={material_name}",
        "*ELASTIC",
        f"{E:.6e}, {nu}",
        "*SHELL SECTION, ELSET=Body, MATERIAL=" + material_name,
        "0.002",
        "*STEP",
        "*STATIC",
        "*BOUNDARY",
        "1, 1, 6, 0.0",
        "*END STEP",
    ]
    path.write_text("\n".join(lines) + "\n")
    return path


def export_abaqus_inp(vertices: np.ndarray, faces: np.ndarray, path: str | Path) -> Path:
    """Abaqus-style INP (similar structure)."""
    path = Path(path)
    lines = ["*Heading", "vehicle_dynamics Abaqus export", "*Node"]
    for i, v in enumerate(vertices, start=1):
        lines.append(f"{i}, {v[0]:.8e}, {v[1]:.8e}, {v[2]:.8e}")
    lines.append("*Element, type=S3, elset=Body")
    for i, f in enumerate(faces, start=1):
        lines.append(f"{i}, {f[0]+1}, {f[1]+1}, {f[2]+1}")
    lines += [
        "*Material, name=Steel",
        "*Elastic",
        "210000000000, 0.3",
        "*Shell Section, elset=Body, material=Steel",
        "0.002,",
        "*Step, name=Static",
        "*Static",
        "*Boundary",
        "1, ENCASTRE",
        "*End Step",
    ]
    path.write_text("\n".join(lines) + "\n")
    return path


def export_code_aster_mesh(vertices: np.ndarray, faces: np.ndarray, path: str | Path) -> Path:
    """Minimal Code_Aster .mail style text."""
    path = Path(path)
    lines = ["TITRE", "vehicle_dynamics", "FINSF", "COOR_3D"]
    for i, v in enumerate(vertices, start=1):
        lines.append(f"N{i} {v[0]:.8e} {v[1]:.8e} {v[2]:.8e}")
    lines.append("FINSF")
    lines.append("TRIA3")
    for i, f in enumerate(faces, start=1):
        lines.append(f"M{i} N{f[0]+1} N{f[1]+1} N{f[2]+1}")
    lines.append("FINSF")
    lines.append("FIN")
    path.write_text("\n".join(lines) + "\n")
    return path


def export_vtk(vertices: np.ndarray, faces: np.ndarray, path: str | Path) -> Path:
    path = Path(path)
    lines = [
        "# vtk DataFile Version 3.0",
        "vehicle_dynamics",
        "ASCII",
        "DATASET POLYDATA",
        f"POINTS {len(vertices)} float",
    ]
    for v in vertices:
        lines.append(f"{v[0]} {v[1]} {v[2]}")
    lines.append(f"POLYGONS {len(faces)} {len(faces)*4}")
    for f in faces:
        lines.append(f"3 {f[0]} {f[1]} {f[2]}")
    path.write_text("\n".join(lines) + "\n")
    return path
