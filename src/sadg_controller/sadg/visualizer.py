from logging import getLogger
from typing import List

import matplotlib.pyplot as plt
import networkx as nx
from matplotlib.figure import Figure

from sadg_controller.sadg.sadg import SADG

logger = getLogger(__name__)


def init_sadg_visualization(sadg: SADG) -> None:
    """Visualize the SADG."""

    G = nx.DiGraph()

    nodes = []
    colors = []

    # Add nodes to graph
    for agent_id, vertices in sadg.vertices.items():

        for idx, vertex in enumerate(vertices):

            colors.append(vertex.color)

            v_agent_id = int(agent_id.replace("agent", ""))
            v_name = vertex.get_shorthand()
            v_data = {"pos": [idx, v_agent_id], "color": vertex.color}
            nodes.append((v_name, v_data))
    G.add_nodes_from(nodes)

    # Add edges to graph
    edges = []
    for agent_id, vertices in sadg.vertices.items():

        for idx, vertex in enumerate(vertices):

            # Add regular dependencies
            if vertex.has_next():
                v_tail = vertex.get_shorthand()
                v_head = vertex.get_next().get_shorthand()
                edges.append((v_tail, v_head))

            v_head = vertex.get_shorthand()
            # Add active amd inactive dependencies
            for dependency in vertex.dependencies:
                if dependency.is_active():
                    v_tail = dependency.get_tail().get_shorthand()
                    edges.append((v_tail, v_head))

                if not dependency.is_active():
                    v_tail = dependency.get_tail().get_shorthand()
                    edges.append((v_tail, v_head))

    G.add_edges_from(edges)

    pos = nx.get_node_attributes(G, "pos")

    options = {
        "edge_color": "grey",
        "node_size": 200,
        "alpha": 0.4,
        "width": 1,
        "with_labels": True,
    }

    plt.ion()
    fig, ax = plt.subplots()

    nx.draw_networkx(G, pos=pos, ax=ax, node_color=colors, **options)

    plt.title("Switchable Action Dependency Graph")

    return fig, G


def update_sadg_visualization(G: nx.DiGraph, sadg: SADG, fig: Figure):

    options = {
        "edge_color": "grey",
        "node_size": 200,
        "alpha": 0.4,
        "width": 1,
        "with_labels": True,
    }

    nodes = update_status(G.nodes(data=True), sadg)
    pos = nx.get_node_attributes(G, "pos")
    colors = list(nx.get_node_attributes(G, "color").values())
    edges = G.edges(data=True)
    G.update(edges, nodes)
    plt.clf()
    nx.draw_networkx(G, pos=pos, node_color=colors, **options)
    fig.canvas.draw()
    fig.canvas.flush_events()


def update_status(nodes: List, sadg: SADG) -> List:
    for _, vertices in sadg.vertices.items():
        for vertex in vertices:
            nodes[vertex.get_shorthand()]["color"] = vertex.color
    return nodes
