#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion

from sadg_controller.comms import Comms
from sadg_controller.core.sadg_compiler import sadg_compiler
from sadg_controller.core.se_adg_compiler import se_adg_compiler
from sadg_controller.mapf.problem import MAPFProblem
from sadg_controller.mapf.roadmap import Roadmap
from sadg_controller.sadg.vertex import Vertex


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

    agents = ["agent1", "agent2"]

    comms_list = [Comms(agent_id) for agent_id in agents]

    del se_adg

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():

        for agent_id, comms in zip(agents, comms_list):
            
            vertex: Vertex = sadg.get_agent_vertex(agent_id)
            
            rospy.loginfo(f"{agent_id}: {vertex.get_start_loc()} -> {vertex.get_goal_loc()}")
            
            comms.publish(Pose(Point(1,2,0), Quaternion(0,0,0,1)))

        rate.sleep()


if __name__ == "__main__":

    roadmap_name = "warehouse"
    roadmap_file = f"/home/alex/github_repos/sadg-controller/data/roadmaps/{roadmap_name}/roadmap.csv"
    dimensions_file = f"/home/alex/github_repos/sadg-controller/data/roadmaps/{roadmap_name}/dimensions.yaml"

    agv_count = 2

    controller(roadmap_file, dimensions_file, agv_count)
