"""Global stiffness assembly."""

from __future__ import annotations

import numpy as np
from .node import Node
from .beam import BeamElement
from .stiffness import global_stiffness


class Model:
    """Collection of nodes and beam elements."""

    def __init__(self):
        self.nodes: list[Node] = []
        self.elements: list[BeamElement] = []

    def add_node(self, x: float, y: float, z: float) -> Node:
        n = Node(id=len(self.nodes), x=x, y=y, z=z)
        self.nodes.append(n)
        return n

    def add_beam(
        self, node_i: Node, node_j: Node, material, section
    ) -> BeamElement:
        e = BeamElement(
            id=len(self.elements),
            node_i=node_i,
            node_j=node_j,
            material=material,
            section=section,
        )
        self.elements.append(e)
        return e

    @property
    def ndof(self) -> int:
        return 6 * len(self.nodes)

    def assemble_stiffness(self) -> np.ndarray:
        K = np.zeros((self.ndof, self.ndof))
        for elem in self.elements:
            ke = global_stiffness(elem)
            dofs = np.concatenate([elem.node_i.dof_indices(), elem.node_j.dof_indices()])
            for a in range(12):
                for b in range(12):
                    K[dofs[a], dofs[b]] += ke[a, b]
        return K
