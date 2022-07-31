import matplotlib.pyplot as plt
import networkx as nx

from sadg_controller.mapf.roadmap import Roadmap


def create_roadmap_graph(roadmap: Roadmap) -> nx.Graph:
    """Converts a roadmap to a networkx graph."""

    roadmap_array = roadmap.array
    dimensions = roadmap.dimensions["dimensions"]

    x_offset = dimensions["x_offset"]
    y_offset = dimensions["y_offset"]
    resolution = dimensions["resolution"]

    graph = nx.Graph()

    nodes = []
    edges = []

    for row_idx, row in enumerate(roadmap_array):
        for col_idx, cell in enumerate(row):
            if int(cell) in [0, 1]:
                print(f"{row_idx},{col_idx}")
                x = col_idx * resolution + x_offset
                y = row_idx * resolution + y_offset

                node_id = f"{row_idx},{col_idx}"
                nodes.append((node_id, {"pos": [x, y]}))

                # Added edges to node below
                if int(roadmap_array[row_idx - 1][col_idx]) in [0, 1]:
                    node_below_id = f"{row_idx-1},{col_idx}"
                    edges.append((node_id, node_below_id))

                # Added edges to node to left
                if int(roadmap_array[row_idx][col_idx - 1]) in [0, 1]:
                    node_left_id = f"{row_idx},{col_idx-1}"
                    edges.append((node_id, node_left_id))

    graph.add_nodes_from(nodes)
    graph.add_edges_from(edges)

    return graph


if __name__ == "__main__":

    roadmap = Roadmap("test")
    graph = create_roadmap_graph(roadmap)

    pos = nx.get_node_attributes(graph, "pos")

    options = {
        "node_color": "black",
        "node_size": 10,
        "width": 2,
    }

    fig, ax = plt.subplots()
    nx.draw_networkx(graph, pos=pos, with_labels=False, ax=ax, **options)
    ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.grid()
    ax.set_aspect("equal")
    plt.show()
