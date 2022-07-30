#!/usr/bin/env python3

from logging import getLogger

import rospy
from geometry_msgs.msg import Point, Pose, Quaternion

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

        self.pose = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))

    def start(self, rate: int = 10):
        """Start."""
        rate = rospy.Rate(rate)
        while not rospy.is_shutdown():

            self.publish_current_pose()
            rate.sleep()

    def callback(self, pose: Pose) -> None:
        """Callback for subscriber to goal position.

        Args:
            pose: Goal pose passed in the message.
        """
        rospy.loginfo(f"{self.ns} : {parse_pose(pose)}")
        self.pose = pose

    def publish_current_pose(self) -> None:
        """Publish current pose."""
        rospy.logdebug(f"{self.pub_link}: Publishing: {parse_pose(self.pose)}")
        self.publisher.publish(self.pose)


if __name__ == "__main__":
    Agent().start(10)
