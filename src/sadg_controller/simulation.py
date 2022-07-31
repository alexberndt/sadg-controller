#!/usr/bin/env python3

import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
import rospy

from sadg_controller.canvas.agent import Agent
from sadg_controller.canvas.roadmap import create_roadmap_graph
from sadg_controller.mapf.roadmap import Roadmap


def simulation():

    rospy.init_node("simulation")
    rospy.loginfo("Starting up the simulation ...")

    roadmap_name = rospy.get_param("~roadmap_name", "test")
    agent_count = rospy.get_param("~agent_count", 20)

    roadmap = Roadmap(roadmap_name)
    graph = create_roadmap_graph(roadmap)

    plt.ion()
    fig, ax = plt.subplots()
    ax.grid(True)
    pos = nx.get_node_attributes(graph, "pos")

    options = {
        "node_color": "black",
        "node_size": 10,
        "width": 2,
    }
    nx.draw_networkx(graph, pos=pos, with_labels=False, ax=ax, **options)
    ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    ax.set_aspect("equal")

    colors = plt.cm.rainbow(np.arange(agent_count) / agent_count)
    agent_ids = [f"agent{id}" for id in range(agent_count)]
    _ = [Agent(id, ax, color) for id, color in zip(agent_ids, colors)]

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():

        fig.canvas.draw()
        fig.canvas.flush_events()
        rate.sleep()


if __name__ == "__main__":
    simulation()
