#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, Pose, Quaternion

from sadg_controller.comms import Comms
from sadg_controller.mapf.problem import MAPFProblem
from sadg_controller.mapf.roadmap import Roadmap
from sadg_controller.sadg.compiler import compile_sadg
from sadg_controller.sadg.status import Status
from sadg_controller.sadg.visualizer import (
    init_sadg_visualization,
    update_sadg_visualization,
)


def controller():
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

    Continuous feedback loop:
        For each agent, if agent:
            1. Can execute current vertex, send new goal
            2. Has no next vertex, agent has reached goal
            3. Current vertex is blocked by dependencies,
                no new goal is sent, and agent waits at
                current location.
    """

    rospy.init_node("controller")
    rospy.loginfo("Starting up the controller ...")

    roadmap_name = rospy.get_param("~roadmap_name")
    agent_count = rospy.get_param("~agent_count")
    visualize_sadg = rospy.get_param("~visualize_sadg", False)
    ecbs_w = rospy.get_param("~ecbs_sub_factor", 1.8)

    roadmap = Roadmap(roadmap_name)

    starts = roadmap.random_locations(agent_count)
    goals = roadmap.random_locations(agent_count)

    problem = MAPFProblem(roadmap, starts, goals)
    plan = problem.solve(suboptimality_factor=ecbs_w)

    sadg = compile_sadg(plan)
    fig, G = init_sadg_visualization(sadg) if visualize_sadg else None

    agent_ids = [f"agent{id}" for id in range(agent_count)]
    comms = [Comms(id, sadg.get_agent_vertex(id)) for id in agent_ids]

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():

        # Loop through each agent and communications
        for comm in comms:

            # Get vertex which agent is currently busy with
            curr_vertex = comm.get_curr_vertex()

            # Check if agent can start executing tasks in vertex.
            # If so, publish new goal location to agent.
            if curr_vertex.can_execute() and curr_vertex.status == Status.STAGED:

                goal = curr_vertex.get_goal_loc()
                goal_pose = Pose(Point(goal.x, goal.y, 0), Quaternion(0, 0, 0, 1))
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

            rospy.logwarn(f"{comm.get_agent_id()} : {msg}")

        rospy.loginfo("--------------------------------------------------")

        if visualize_sadg:
            update_sadg_visualization(G, sadg, fig)

        rate.sleep()


if __name__ == "__main__":
    controller()
