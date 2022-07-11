from typing import Dict, List

import matplotlib.pyplot as plt
import networkx as nx

from sadg_controller.sadg.dependency import Dependency
from sadg_controller.sadg.dependency_switch import DependencySwitch
from sadg_controller.sadg.vertex import Vertex
from sadg_controller.se_adg.se_adg import SE_ADG


class SADG:
    def __init__(
        self,
        vertices: Dict[str, List[Vertex]],
        regular_deps: Dict[str, List[Dependency]],
        switchable_deps: Dict[str, List[DependencySwitch]],
    ) -> None:
        self.vertices = vertices
        self.regular_deps = regular_deps
        self.switchable_deps = switchable_deps

    def get_SE_ADG(self, b_vec: List[bool]) -> SE_ADG:
        """Get the SE-ADG for a given boolean vector.

        Args:
            b_vec: Vector of booleans.
        """

        active_dependencies = self.regular_deps

        for agent_id, dep_switch in self.switchable_deps:

            # TODO: use b_vec as input
            active_dep = dep_switch.get_active()

            active_dependencies[agent_id].append(active_dep)

        return SE_ADG(self.vertices, active_dependencies)

    def plot(self, title: str) -> None:
        """Plots the SADG."""

        G = nx.DiGraph()

        # Add vertices
        nodes = []
        for agent_id, vertices in self.vertices.items():

            for idx, v in enumerate(vertices):
                v_name = f"v_{agent_id.split('_')[-1]}_{idx}"
                v_data = {"color": "red"}  # v.get_status()
                nodes.append((v_name, v_data))

        G.add_nodes_from(nodes)

        # Add dependencies
        edges = []
        for agent_id, dependencies in self.regular_deps.items():
            for idx, e in enumerate(dependencies):

                e_name = str(e)
                edges.append(e_name)

        nx.draw(G)
        plt.show()

        print("test")
