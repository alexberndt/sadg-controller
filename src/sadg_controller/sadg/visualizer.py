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


class Visualizer:
    def __init__(self, sadg: SADG) -> None:

        self.sadg = sadg
        self.G = nx.DiGraph()

        nodes = []

        # Add nodes to graph
        for agent_id, vertices in self.sadg.vertices.items():

            for idx, vertex in enumerate(vertices):
                v_agent_id = int(agent_id.replace("agent", ""))
                v_name = vertex.get_shorthand()
                v_data = {"pos": [idx, v_agent_id], "color": vertex.color}
                nodes.append((v_name, v_data))
        self.G.add_nodes_from(nodes)

        # Add edges to graph
        edges = []
        edge_colors = []
        for agent_id, vertices in self.sadg.vertices.items():

            for idx, vertex in enumerate(vertices):

                # Add regular dependencies
                if vertex.has_next():
                    v_tail = vertex.get_shorthand()
                    v_head = vertex.get_next().get_shorthand()
                    edges.append((v_tail, v_head, {"color": "#222"}))

                v_head = vertex.get_shorthand()
                # Add active amd inactive dependencies
                for dependency in vertex.dependencies:
                    if dependency.is_active():
                        v_tail = dependency.get_tail().get_shorthand()
                        edges.append((v_tail, v_head, {"color": "#222"}))

                    if not dependency.is_active():
                        v_tail = dependency.get_tail().get_shorthand()
                        edges.append((v_tail, v_head, {"color": "#ddd"}))

        self.G.add_edges_from(edges)
        self.pos = nx.get_node_attributes(self.G, "pos")

        plt.ion()
        self.fig, self.ax = plt.subplots()

        node_colors = list(nx.get_node_attributes(self.G, "color").values())
        edge_colors = list(nx.get_edge_attributes(self.G, "color").values())
        nx.draw_networkx(
            self.G,
            pos=self.pos,
            node_color=node_colors,
            edge_color=edge_colors,
            **OPTIONS
        )
        plt.title(TITLE)

    def refresh(self) -> None:
        """Refresh the SADG visualization.

        Updates the node colors for each vertex based on
        the vertex status.
        """

        # Update color of each vertex based on status
        nodes = self.update_node_status()

        # Update colors
        node_colors = list(nx.get_node_attributes(self.G, "color").values())
        edge_colors = list(nx.get_edge_attributes(self.G, "color").values())
        edges = self.G.edges(data=True)
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
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def update_node_status(self) -> List:
        """Update node colors based on vertex statuses.

        Returns:
            Updated list of notes with colors based on vertex
                statuses.
        """
        nodes = self.G.nodes(data=True)
        for _, vertices in self.sadg.vertices.items():
            for vertex in vertices:
                nodes[vertex.get_shorthand()]["color"] = vertex.color
        return nodes
