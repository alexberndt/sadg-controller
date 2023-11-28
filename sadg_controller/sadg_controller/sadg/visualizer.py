# sadg-controller
# Copyright (c) 2023 Alexander Berndt, Robert Bosch GmbH
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as published
# by the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Affero General Public License for more details.
#
# You should have received a copy of the GNU Affero General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

from logging import getLogger
from typing import List

import matplotlib.pyplot as plt
import networkx as nx

from sadg_controller.sadg.sadg import SADG

logger = getLogger(__name__)

OPTIONS = {
    "node_size": 200,
    "alpha": 0.4,
    "width": 1,
    "with_labels": True,
}
TITLE = "Switchable Action Dependency Graph"


GREEN = "#008000"
LIGHT_GREY = "#D3D3D3"
BLACK = "#000000"


class Visualizer:
    def __init__(self, sadg: SADG) -> None:

        self.sadg = sadg
        self.G = nx.DiGraph()

        nodes = []

        # Add nodes to graph
        for agent_id, vertices in self.sadg.vertices_by_agent.items():

            for idx, vertex in enumerate(vertices):
                v_agent_id = int(agent_id.replace("agent", ""))
                v_name = vertex.get_shorthand()
                v_data = {"pos": [idx, v_agent_id], "color": vertex.color}
                nodes.append((v_name, v_data))
        self.G.add_nodes_from(nodes)

        # Add edges to graph
        edges = []
        for agent_id, vertices in self.sadg.vertices_by_agent.items():

            for idx, vertex in enumerate(vertices):

                # Add regular dependencies
                if vertex.has_next():
                    v_tail = vertex.get_shorthand()
                    v_head = vertex.get_next().get_shorthand()
                    edges.append((v_tail, v_head, {"color": BLACK}))

                v_head = vertex.get_shorthand()

                # Add active and inactive dependencies
                for dependency in vertex.get_dependencies():

                    color = GREEN if dependency.is_active() else LIGHT_GREY
                    v_tail = dependency.get_tail().get_shorthand()
                    edges.append((v_tail, v_head, {"color": color}))

        self.G.add_edges_from(edges)
        self.pos = nx.get_node_attributes(self.G, "pos")

        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.plot_graph()

    def refresh(self) -> None:
        """Refresh the SADG visualization.

        Updates the node colors for each vertex based on
        the vertex status.
        """

        self.plot_graph()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def plot_graph(self) -> None:
        # Update color of each vertex based on status
        nodes = self.update_node_status()
        edges = self.update_edge_colors()

        # Update colors
        node_colors = list(nx.get_node_attributes(self.G, "color").values())
        edge_colors = list(nx.get_edge_attributes(self.G, "color").values())

        # edges = self.G.edges(data=True)
        self.G.update(edges, nodes)

        # Redraw the figure
        plt.clf()
        nx.draw_networkx(
            self.G,
            pos=self.pos,
            node_color=node_colors,
            edge_color=edge_colors,
            **OPTIONS
        )
        plt.title(TITLE)

    def update_node_status(self) -> List:
        """Update node colors based on vertex statuses.

        Returns:
            Updated list of notes with colors based on vertex
                statuses.
        """
        nodes = self.G.nodes(data=True)
        for _, vertices in self.sadg.vertices_by_agent.items():
            for vertex in vertices:
                nodes[vertex.get_shorthand()]["color"] = vertex.color
        return nodes

    def update_edge_colors(self) -> List:
        """Update node colors based on vertex statuses.

        Returns:
            Updated list of notes with colors based on vertex
                statuses.
        """
        edges = self.G.edges(data=True)
        for _, dep_groups in self.sadg.get_switchable_dep_groups().items():

            for dep_group in dep_groups:

                for dep_switch in dep_group.get_dependencies():

                    # Color active dependency green
                    active_dep = dep_switch.get_active()
                    tail_str = active_dep.get_tail().get_shorthand()
                    head_str = active_dep.get_head().get_shorthand()

                    # TODO: update edge color
                    # edges[(tail_str, head_str)]["color"] = GREEN

                    # Color active dependency grey
                    inactive_dep = dep_switch.get_inactive()
                    if inactive_dep is not None:
                        tail_str = inactive_dep.get_tail().get_shorthand()
                        head_str = inactive_dep.get_head().get_shorthand()

                    del tail_str
                    del head_str

                    # TODO: update edge color
                    # edges[(tail_str, head_str)]["color"] = LIGHT_GREY

        return edges
