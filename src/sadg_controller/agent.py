#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from logging import getLogger

logger = getLogger(__name__)


class Agent:
    def __init__(self) -> None:
        rospy.init_node("agent")
        self.ns = rospy.get_param("~agent_ns")
        self.uuid = rospy.get_param("~uuid")

        self.subscriber = rospy.Subscriber(f"/{self.ns}/from", String, self.callback)


    def start(self, rate: int = 5):

        rate = rospy.Rate(rate)
        while not rospy.is_shutdown():

            rospy.loginfo(f"Agent {self.uuid} running ...")
            rate.sleep()

    
    def callback(self, data):
        rospy.loginfo(f"Callback ns: '{data}'")
        

if __name__ == "__main__":
    Agent().start(1)
