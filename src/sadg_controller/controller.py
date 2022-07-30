#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion

from sadg_controller.comms import Comms
from sadg_controller.core.sadg_compiler import sadg_compiler
from sadg_controller.core.se_adg_compiler import se_adg_compiler
from sadg_controller.mapf.problem import MAPFProblem
from sadg_controller.mapf.roadmap import Roadmap


def controller(roadmap_file: str, dimensions_file: str, agv_count: int):

    rospy.init_node("controller")
    rospy.loginfo("Hello from the controller ...")

    roadmap = Roadmap(roadmap_file, dimensions_file)

    starts = roadmap.random_locations(agv_count)
    goals = roadmap.random_locations(agv_count)

    problem = MAPFProblem(roadmap, starts, goals)
    plan = problem.solve(suboptimality_factor=1.8)

    sadg = sadg_compiler(plan)
    se_adg = se_adg_compiler(plan)

    agents_comms = [
        Comms("agent0", sadg.get_agent_vertex("agent0")),
        Comms("agent1", sadg.get_agent_vertex("agent1")),
        Comms("agent2", sadg.get_agent_vertex("agent2")),
        Comms("agent3", sadg.get_agent_vertex("agent3")),
        Comms("agent4", sadg.get_agent_vertex("agent4")),
        Comms("agent5", sadg.get_agent_vertex("agent5")),
        Comms("agent6", sadg.get_agent_vertex("agent6")),
        Comms("agent7", sadg.get_agent_vertex("agent7")),
    ]
    
    del se_adg

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        
        # Loop through each agent and communications
        for agent_comms in agents_comms:

            v = agent_comms.get_curr_vertex()
            

            # rospy.logdebug(f"Vertex status: {v.status}") 
            # rospy.logdebug(f"Vertex next: {v.get_next()}")  
            # rospy.logdebug(f"Can execute?: {v.can_execute()}")

            msg = f"{agent_comms.get_agent_id()} : " 

            if not v.has_next():
                msg += f"Last vertex reached"
            elif v.can_execute():
                goal = v.get_goal_loc()
                agent_comms.publish(Pose(Point(goal.x,goal.y,0), Quaternion(0,0,0,1)))
                msg += f"Goal = {v.get_goal_loc()}"
            else:
                msg += "Blocked by dependency"

            rospy.logwarn(msg)

        rospy.loginfo("--------------------------------------------------")
        rate.sleep()


if __name__ == "__main__":

    roadmap_name = "test"
    roadmap_file = f"/home/alex/github_repos/sadg-controller/data/roadmaps/{roadmap_name}/roadmap.csv"
    dimensions_file = f"/home/alex/github_repos/sadg-controller/data/roadmaps/{roadmap_name}/dimensions.yaml"

    agv_count = 8

    controller(roadmap_file, dimensions_file, agv_count)
