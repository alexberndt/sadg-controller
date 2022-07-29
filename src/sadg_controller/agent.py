#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from logging import getLogger
from sadg_controller.comms import parse_pose

logger = getLogger(__name__)


class Agent:
    def __init__(self) -> None:
        rospy.init_node("agent")
        self.ns = rospy.get_param("~agent_ns")
        self.uuid = rospy.get_param("~uuid")

        self.sub_link = f"/{self.ns}/goal"
        self.subscriber = rospy.Subscriber(self.sub_link, Pose, self.callback)

        self.pub_link = f"/{self.ns}/current"
        self.publisher = rospy.Publisher(self.pub_link, Pose, queue_size=1000)


    def start(self, rate: int = 30):
        rate = rospy.Rate(rate)
        while not rospy.is_shutdown():

            rospy.loginfo(f"Agent {self.uuid} running ...")
            rate.sleep()

    
    def callback(self, pose: Pose) -> None:
        """Callback for subscriber. """
        rospy.loginfo(f"{self.sub_link}: Callback: {parse_pose(pose)}")


    def publish(self, pose: Pose) -> None:
        """Publish Pose"""
        rospy.loginfo(f"{self.pub_link}: Publishing: {parse_pose(pose)}")
        self.publisher.publish(pose)


if __name__ == "__main__":
    Agent().start(30)
