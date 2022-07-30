#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, Pose, Quaternion

from sadg_controller.comms import Comms
from sadg_controller.core.sadg_compiler import sadg_compiler
from sadg_controller.mapf.problem import MAPFProblem
from sadg_controller.mapf.roadmap import Roadmap
from sadg_controller.sadg.status import Status


def controller(roadmap_name: str = None, agent_count: int = None, ecbs_w: float = 1.8):

    rospy.init_node("controller")
    rospy.loginfo("Hello from the controller ...")

    roadmap_name = rospy.get_param("~roadmap_name")
    agent_count = rospy.get_param("~agent_count")
    ecbs_w = rospy.get_param("~ecbs_sub_factor", 1.8)

    roadmap = Roadmap(roadmap_name)

    starts = roadmap.random_locations(agent_count)
    goals = roadmap.random_locations(agent_count)

    problem = MAPFProblem(roadmap, starts, goals)
    plan = problem.solve(suboptimality_factor=ecbs_w)

    sadg = sadg_compiler(plan)

    agent_ids = [f"agent{id}" for id in range(agent_count)]
    agents_comms = [Comms(id, sadg.get_agent_vertex(id)) for id in agent_ids]

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():

        # Loop through each agent and communications
        for agent_comms in agents_comms:

            v = agent_comms.get_curr_vertex()

            # rospy.logdebug(f"Vertex status: {v.status}")
            # rospy.logdebug(f"Vertex next: {v.get_next()}")
            # rospy.logdebug(f"Can execute?: {v.can_execute()}")

            msg = f"{agent_comms.get_agent_id()} : "

            if v.can_execute() and v.status == Status.STAGED:
                goal = v.get_goal_loc()
                agent_comms.publish(
                    Pose(Point(goal.x, goal.y, 0), Quaternion(0, 0, 0, 1))
                )
                msg += f"Goal = {v.get_goal_loc()}"
            elif not v.has_next():
                msg += "Last vertex reached"
            else:
                blocking_vertices = v.get_blocking_vertices()
                msg += f"Blocked by dependency: {blocking_vertices}"

            rospy.logwarn(msg)

        rospy.loginfo("--------------------------------------------------")
        rate.sleep()


if __name__ == "__main__":

    roadmap_name = "warehouse"
    agent_count = 40

    controller(roadmap_name, agent_count)
