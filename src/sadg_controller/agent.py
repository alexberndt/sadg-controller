#!/usr/bin/env python3

import rospy
from logging import getLogger

logger = getLogger(__name__)


class Agent:
    def __init__(self) -> None:
        rospy.init_node("agent")
        self.ns = rospy.get_param("~agent_ns")
        self.uuid = rospy.get_param("~uuid")


    def start(self, rate: int = 10):

        rate = rospy.Rate(rate)
        while not rospy.is_shutdown():

            rospy.loginfo(f"Agent agent_{self.uuid} running ...")
            rate.sleep()
        

if __name__ == "__main__":
    Agent().start(15)
