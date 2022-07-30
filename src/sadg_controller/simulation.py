#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import rospy

from sadg_controller.canvas.agent import Agent

# from sadg_controller.mapf.roadmap import Roadmap


def simulation():

    rospy.init_node("simulation")
    rospy.loginfo("Starting up the simulation ...")

    # roadmap_name = rospy.get_param("~roadmap_name")
    agent_count = rospy.get_param("~agent_count")

    # roadmap = Roadmap(roadmap_name)

    # Plot
    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.grid(True)
    ax.set_xlim((-20, 20))
    ax.set_ylim((-20, 40))
    ax.set_aspect("equal")

    colors = plt.cm.rainbow(np.arange(agent_count) / agent_count)
    agent_ids = [f"agent{id}" for id in range(agent_count)]
    _ = [Agent(id, ax, color) for id, color in zip(agent_ids, colors)]

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():

        fig.canvas.draw()
        fig.canvas.flush_events()
        rate.sleep()


if __name__ == "__main__":
    simulation()
