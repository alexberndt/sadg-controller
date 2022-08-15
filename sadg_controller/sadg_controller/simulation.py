#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
from rclpy.node import Node

from sadg_controller.canvas.agent import Agent
from sadg_controller.canvas.roadmap import plot_roadmap_graph
from sadg_controller.mapf.roadmap import Roadmap


class Simulation(Node):

    def __init__(self, time_step: float = 0.05) -> None:
        super().__init__("simulation")
        self.get_logger().info("Starting up the simulation ...")

        self.time_step = time_step

        roadmap_name = self.declare_parameter('roadmap_name', 'test').value
        agent_count = self.declare_parameter('agent_count', 20).value

        plt.ion()
        self.fig, ax = plt.subplots()
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

    def start(self) -> None:
        self.create_timer(self.time_step, self.simulation_task)

    def simulation_task(self) -> None:
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


if __name__ == "__main__":
    Simulation().start()
