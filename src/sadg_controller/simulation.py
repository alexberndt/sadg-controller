#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import rospy

from sadg_controller.canvas.agent import Agent
from sadg_controller.canvas.roadmap import plot_roadmap_graph
from sadg_controller.mapf.roadmap import Roadmap


def simulation():

    rospy.init_node("simulation")
    rospy.loginfo("Starting up the simulation ...")

    roadmap_name = rospy.get_param("~roadmap_name", "test")
    agent_count = rospy.get_param("~agent_count", 20)

    plt.ion()
    fig, ax = plt.subplots()
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.grid(True)
    plt.title("Roadmap with agents")
    ax.set_aspect("equal")

    roadmap = Roadmap(roadmap_name)
    plot_roadmap_graph(roadmap, ax)

    colors = plt.cm.rainbow(np.arange(agent_count) / agent_count)
    agent_ids = [f"agent{id}" for id in range(agent_count)]
    _ = [Agent(id, ax, color) for id, color in zip(agent_ids, colors)]

    rate = rospy.Rate(15)
    while not rospy.is_shutdown():

        fig.canvas.draw()
        fig.canvas.flush_events()
        rate.sleep()


if __name__ == "__main__":
    simulation()
