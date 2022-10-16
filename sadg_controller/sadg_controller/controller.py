#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion
from rclpy.node import Node

from sadg_controller.comms import Comms
from sadg_controller.mapf.problem import MAPFProblem
from sadg_controller.mapf.roadmap import Roadmap
from sadg_controller.sadg.compiler import compile_sadg
from sadg_controller.sadg.status import Status
from sadg_controller.sadg.visualizer import Visualizer


class Controller(Node):
    def __init__(self) -> None:
        """SADG Controller.

        ROS node which initializes a SADG controller based on a roadmap and
        randomized agent goal/start locations. Once initialized, communication
        is started with each agent and goal locations sent to each agent based
        on the SADG dependency graph.

        Initialization:
            1. Initialize a roadmap given a `roadmap_name`
            2. Assign randomized start/goal locations for `agent_count` agents
            3. Define and solve the MAPF problem
            4. Compile the SADG from the MAPF solution
            5. Initialize Comms to the `agent_count` agents
        """
        super().__init__("controller")
        self.logger = self.get_logger()

        self.logger.info("Starting up the controller ...")

        self.time_step = self.declare_parameter("time_step", 2.0).value
        roadmap_path = self.declare_parameter("roadmap_path", "/tmp").value
        agent_count = self.declare_parameter("agent_count", 1).value
        self.visualize_sadg = self.declare_parameter("visualize_sadg", False).value
        ecbs_w = self.declare_parameter("ecbs_sub_factor", 1.8).value

        roadmap = Roadmap(roadmap_path)

        starts = roadmap.random_locations(agent_count)
        goals = roadmap.random_locations(agent_count)

        problem = MAPFProblem(roadmap, starts, goals, self.logger)
        plan = problem.solve(suboptimality_factor=ecbs_w)

        self.sadg = compile_sadg(plan, self.logger)
        self.sadg_visualizer = Visualizer(self.sadg) if self.visualize_sadg else None

        agent_ids = [f"agent{id}" for id in range(agent_count)]
        self.comms = [
            Comms(self, id, self.sadg.get_agent_first_vertex(id)) for id in agent_ids
        ]

    def start(self) -> None:
        """Start controller."""
        self.create_timer(self.time_step, self.controller_task)

    def controller_task(self) -> None:
        """Controller task.

        Continuous feedback loop:
            For each agent, if agent:
                1. Can execute current vertex, send new goal
                2. Has no next vertex, agent has reached goal
                3. Current vertex is blocked by dependencies,
                    no new goal is sent, and agent waits at
                    current location.
        """
        # Loop through each agent and communications
        for comm in self.comms:

            # Get vertex which agent is currently busy with
            curr_vertex = comm.get_curr_vertex()

            # Check if agent can start executing tasks in vertex.
            # If so, publish new goal location to agent.
            if curr_vertex.can_execute() and curr_vertex.status == Status.STAGED:

                goal = curr_vertex.get_goal_loc()
                goal_pose = Pose(
                    position=Point(x=goal.x, y=goal.y, z=0.0),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                )
                comm.publish_pose_goal(goal_pose)
                curr_vertex.set_status(Status.IN_PROGRESS)
                msg = f"Goal = {curr_vertex.get_goal_loc()}"

            # If vertex is being executed still
            elif curr_vertex.status == Status.IN_PROGRESS:
                msg = "In progress"

            # If there is no next vertex, agent has reached end goal
            elif not curr_vertex.has_next():
                msg = "Reached final goal"

            # If there are next vertices, agent is blocked by dependencies
            # and must wait before proceeding
            else:
                msg = "Blocked by dependencies"

            self.logger.warn(f"{comm.get_agent_id()} : {msg}")

        self.logger.info("--------------------------------------------------")

        self.logger.info("Running RHC optimization ...")

        

        self.logger.info("--------------------------------------------------")

        if self.visualize_sadg:
            self.sadg_visualizer.refresh()


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    controller.start()

    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
