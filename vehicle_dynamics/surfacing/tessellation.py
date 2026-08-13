"""LOD tessellation helpers."""
from __future__ import annotations

from .mesh_generator import MeshGenerator, GeneratedMesh


def generate_lods(body, edge_lengths=(0.08, 0.04, 0.02)) -> list[GeneratedMesh]:
    return [MeshGenerator(target_edge_length=e).generate(body) for e in edge_lengths]
