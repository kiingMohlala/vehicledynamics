"""Export CFD cases: OpenFOAM-style, SU2, STL, CGNS-lite JSON."""
from __future__ import annotations

from pathlib import Path
from typing import Any
import json
import numpy as np


def export_stl(vertices: np.ndarray, faces: np.ndarray, path: str | Path) -> Path:
    path = Path(path)
    lines = ["solid vehicle"]
    for f in faces:
        p0, p1, p2 = vertices[f[0]], vertices[f[1]], vertices[f[2]]
        n = np.cross(p1 - p0, p2 - p0)
        nn = np.linalg.norm(n)
        n = n / nn if nn > 1e-15 else np.array([0.0, 0.0, 1.0])
        lines.append(f"  facet normal {n[0]:.6e} {n[1]:.6e} {n[2]:.6e}")
        lines.append("    outer loop")
        for p in (p0, p1, p2):
            lines.append(f"      vertex {p[0]:.6e} {p[1]:.6e} {p[2]:.6e}")
        lines.append("    endloop")
        lines.append("  endfacet")
    lines.append("endsolid vehicle")
    path.write_text("\n".join(lines) + "\n")
    return path


def export_openfoam(vertices: np.ndarray, faces: np.ndarray, case_dir: str | Path, bcs: list | None = None) -> Path:
    """Minimal OpenFOAM polyMesh-like points/faces + boundary dict (engineering export)."""
    case_dir = Path(case_dir)
    mesh_dir = case_dir / "constant" / "polyMesh"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    # points
    pts = ["FoamFile{version 2.0; format ascii; class vectorField; object points;}", f"{len(vertices)}"]
    pts.append("(")
    for v in vertices:
        pts.append(f"({v[0]:.8e} {v[1]:.8e} {v[2]:.8e})")
    pts.append(")")
    (mesh_dir / "points").write_text("\n".join(pts) + "\n")
    # faces
    fc = ["FoamFile{version 2.0; format ascii; class faceList; object faces;}", f"{len(faces)}"]
    fc.append("(")
    for f in faces:
        fc.append(f"3({f[0]} {f[1]} {f[2]})")
    fc.append(")")
    (mesh_dir / "faces").write_text("\n".join(fc) + "\n")
    # boundary
    bc_lines = ["FoamFile{version 2.0; format ascii; class polyBoundaryMesh; object boundary;}", "2", "("]
    bc_lines.append("body { type wall; nFaces %d; startFace 0; }" % len(faces))
    bc_lines.append("defaultFaces { type patch; nFaces 0; startFace %d; }" % len(faces))
    bc_lines.append(")")
    (mesh_dir / "boundary").write_text("\n".join(bc_lines) + "\n")
    if bcs:
        (case_dir / "boundaryConditions.json").write_text(
            json.dumps([{"name": b.name, "type": b.bc_type, "region": b.region, "values": b.values} for b in bcs], indent=2)
        )
    return case_dir


def export_su2(vertices: np.ndarray, faces: np.ndarray, path: str | Path, marker: str = "body") -> Path:
    """SU2 native mesh (ASCII)."""
    path = Path(path)
    lines = [
        "NDIME= 3",
        f"NPOIN= {len(vertices)}",
    ]
    for i, v in enumerate(vertices):
        lines.append(f"{v[0]:.8e} {v[1]:.8e} {v[2]:.8e} {i}")
    lines.append(f"NELEM= {len(faces)}")
    for i, f in enumerate(faces):
        # VTK triangle type 5
        lines.append(f"5 {f[0]} {f[1]} {f[2]} {i}")
    lines.append(f"NMARK= 1")
    lines.append(f"MARKER_TAG= {marker}")
    lines.append(f"MARKER_ELEMS= {len(faces)}")
    for f in faces:
        lines.append(f"5 {f[0]} {f[1]} {f[2]}")
    path.write_text("\n".join(lines) + "\n")
    return path


def export_cgns_meta(vertices: np.ndarray, faces: np.ndarray, path: str | Path) -> Path:
    """CGNS-like JSON metadata (not binary CGNS)."""
    path = Path(path)
    data = {
        "format": "cgns-meta-json",
        "n_points": len(vertices),
        "n_elements": len(faces),
        "element_type": "TRI_3",
        "points_sample": vertices[:5].tolist() if len(vertices) else [],
    }
    path.write_text(json.dumps(data, indent=2))
    return path
