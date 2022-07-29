#!/usr/bin/env python3

import rospy
from logging import getLogger

logger = getLogger(__name__)


class Agent:
    def __init__(self, uuid: str) -> None:
        rospy.init_node("agent")
        self.ns = rospy.get_param("~agent_ns")
        self.uuid = uuid


    def start(self, rate: int = 10):

        rate = rospy.Rate(rate)
        while not rospy.is_shutdown():

            rospy.loginfo(f"Agent agent_{self.uuid} running ...")
            rate.sleep()
        

if __name__ == "__main__":
    Agent("cn3478xk").start(15)
