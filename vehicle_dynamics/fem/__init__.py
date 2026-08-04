"""Phase 8.0 – 3D Euler-Bernoulli beam FEM foundation."""

from .node import Node
from .beam import BeamElement
from .material import Material, steel, aluminum
from .section import Section, circular, rectangular, tube
from .assembler import Model
from .constraints import fix_node, pin_node, apply_force
from .solver import solve_static
from .result import StaticResult

__all__ = [
    "Node",
    "BeamElement",
    "Material",
    "steel",
    "aluminum",
    "Section",
    "circular",
    "rectangular",
    "tube",
    "Model",
    "fix_node",
    "pin_node",
    "apply_force",
    "solve_static",
    "StaticResult",
]
