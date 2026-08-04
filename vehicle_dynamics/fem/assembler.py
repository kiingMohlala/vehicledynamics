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
        self._node_by_tag: dict[str, Node] = {}

    def add_node(
        self, x: float, y: float, z: float, tag: str = ""
    ) -> Node:
        for existing in self.nodes:
            d = np.linalg.norm(
                np.array([x, y, z]) - existing.coords()
            )
            if d < 1e-9:
                raise ValueError(
                    f"Duplicate node near ({x},{y},{z}); "
                    f"existing id={existing.id} tag={existing.tag!r}"
                )
        n = Node(id=len(self.nodes), x=x, y=y, z=z, tag=tag)
        self.nodes.append(n)
        if tag:
            self._node_by_tag[tag] = n
        return n

    def get_node(self, tag: str) -> Node:
        if tag not in self._node_by_tag:
            raise KeyError(f"No node tagged {tag!r}")
        return self._node_by_tag[tag]

    def add_beam(
        self, node_i: Node, node_j: Node, material, section, tag: str = ""
    ) -> BeamElement:
        d = node_j.coords() - node_i.coords()
        if float(np.linalg.norm(d)) < 1e-12:
            raise ValueError(
                f"Refusing zero-length beam between nodes "
                f"{node_i.id} and {node_j.id}"
            )
        e = BeamElement(
            id=len(self.elements),
            node_i=node_i,
            node_j=node_j,
            material=material,
            section=section,
            tag=tag,
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
            dofs = np.concatenate(
                [elem.node_i.dof_indices(), elem.node_j.dof_indices()]
            )
            for a in range(12):
                for b in range(12):
                    K[dofs[a], dofs[b]] += ke[a, b]
        return K
