from logging import getLogger
from typing import List

import matplotlib.pyplot as plt
import networkx as nx
import rospy
from matplotlib.figure import Figure

from sadg_controller.core.geometry import intersects
from sadg_controller.mapf.plan import Plan
from sadg_controller.sadg.dependency import Dependency
from sadg_controller.sadg.dependency_group import DependencyGroup
from sadg_controller.sadg.dependency_switch import DependencySwitch
from sadg_controller.sadg.sadg import SADG
from sadg_controller.sadg.status import Status
from sadg_controller.sadg.vertex import Vertex, loc

logger = getLogger(__name__)


def compile_sadg(P: Plan) -> SADG:  # noqa: C901
    """Algorithm 2: SADG Compiler.

    Compiles a switchable action dependency graph (SADG) from
      a valid MAPF plan.

    Args:
        plan: MAPF plan determined by a MAPF planner.

    Raises:
        PlanContainsDeadlocks.

    Returns:
        SADG from the provided plan.
    """

    # N = len(P.plans.keys())

    # Initialize
    E_regular = {}
    E_switchable = {}
    V_sadg = {}

    # Add sequence of vents for each AGV (Alg. 2, lines 1 - 13)
    for agent_id, P_i in P.plans.items():

        # initialize
        # N_i = len(P_i)
        P_i = iter(P_i)
        V_sadg[agent_id] = []
        E_regular[agent_id] = []
        E_switchable[agent_id] = []
        i = 0

        p = next(P_i)
        v = Vertex(agent_id, p, i, Status.STAGED)
        v_prev = None

        for p_k in P_i:

            v.append_plan_tuple(p_k)

            # Check for spatial exclusivity
            if not intersects(loc(p), loc(p_k)):

                V_sadg[agent_id].append(v)

                if v_prev is not None:
                    E_regular[agent_id].append(Dependency(v_prev, v, active=True))
                    v_prev.set_next(v)

                # reset
                v_prev = v
                p = p_k
                i += 1
                v = Vertex(agent_id, p, i, Status.STAGED)

    # Add switchable dependency pairs (Alg. 2, lines 14 - 24)
    for agent_i, vertices_i in V_sadg.items():
        for agent_j, vertices_j in V_sadg.items():

            if agent_i == agent_j:
                continue

            for k, v_i_k in enumerate(vertices_i):
                for l, v_j_l in enumerate(vertices_j):  # noqa: E741

                    if (
                        v_i_k.get_start_loc() == v_j_l.get_goal_loc()
                        and v_i_k.get_goal_time() <= v_j_l.get_goal_time()
                    ):

                        fwd = Dependency(v_i_k, v_j_l, active=True)
                        v_j_l.add_dependency(fwd)

                        try:
                            v_i_k_minus_1 = vertices_i[k - 1]
                            v_j_l_plus_1 = vertices_j[l + 1]

                            rev = Dependency(v_j_l_plus_1, v_i_k_minus_1, active=False)
                            v_i_k_minus_1.add_dependency(rev)

                            switch = DependencySwitch(fwd, rev)
                            E_switchable[agent_i].append(switch)

                        except IndexError:
                            rospy.logdebug(
                                f"k-1={k - 1}, l+1={l + 1} are not valid indexes."
                                "Cannot construct dependency switch."
                            )
                            E_regular[agent_i].append(fwd)

    # Group dependencies for switching
    E_groups = {}
    for agent_i, switches in E_switchable.items():
        rospy.loginfo(f"Agent: {agent_i}")
        E_groups[agent_i] = []
        for switch in switches:
            rospy.logdebug(
                f"{switch.fwd.tail.get_agent_id()} {switch.fwd.tail.get_vertex_idx()} -> {switch.fwd.head.get_agent_id()} {switch.fwd.head.get_vertex_idx()}"
            )

            if len(E_groups[agent_i]) == 0:
                E_groups[agent_i].append(DependencyGroup(switch))
            else:
                if not E_groups[agent_i][-1].append_switch(switch):
                    E_groups[agent_i].append(DependencyGroup(switch))

    # Create summary
    switch_cnt = 0
    for agent_i, switches in E_switchable.items():
        for _ in switches:
            switch_cnt += 1

    group_cnt = 0
    for agent_i, groups in E_groups.items():
        for _ in groups:
            group_cnt += 1

    rospy.loginfo(f"Switches: {switch_cnt}")
    rospy.loginfo(f"Groups:   {group_cnt}")

    return SADG(V_sadg, E_regular, E_groups)


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


def visualize_sadg(G: nx.DiGraph, sadg: SADG, fig: Figure):

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
