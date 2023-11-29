#!/usr/bin/env python3
# sadg-controller - MAPF execution with Switchable Action Dependency Graphs
# Copyright (c) 2023 Alex Berndt
# Copyright (c) 2023 Niels van Duijkeren, Robert Bosch GmbH
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

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node

from sadg_controller.canvas.agent import Agent
from sadg_controller.canvas.roadmap import plot_roadmap_graph
from sadg_controller.mapf.roadmap import Roadmap


class Simulation(Node):
    def __init__(self) -> None:
        super().__init__("simulation")
        self.get_logger().info("Starting up the simulation ...")

        self.time_step = self.declare_parameter("time_step", 0.05).value

        roadmap_path = self.declare_parameter("roadmap_path", "/tmp").value
        agent_count = self.declare_parameter("agent_count", 20).value

        plt.ion()
        self.fig, ax = plt.subplots()
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.grid(True)
        plt.title("Roadmap with agents")
        ax.set_aspect("equal")

        roadmap = Roadmap(roadmap_path)
        plot_roadmap_graph(roadmap, ax)

        colors = plt.cm.rainbow(np.arange(agent_count) / agent_count)
        agent_ids = [f"agent{id}" for id in range(agent_count)]
        _ = [Agent(self, id, ax, color) for id, color in zip(agent_ids, colors)]

    def start(self) -> None:
        self.create_timer(self.time_step, self.simulation_task)

    def simulation_task(self) -> None:
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    simulation = Simulation()
    simulation.start()

    rclpy.spin(simulation)
    simulation.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
