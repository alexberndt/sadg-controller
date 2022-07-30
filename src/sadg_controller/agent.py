#!/usr/bin/env python3

import math
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
        self.pose_goal = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))

    def start(self, rate: int = 30):
        """Start."""
        rate = rospy.Rate(rate)
        while not rospy.is_shutdown():

            self.move_towards_goal_pose()
            self.publish_current_pose()
            rate.sleep()

    def callback(self, pose_goal: Pose) -> None:
        """Callback for subscriber to goal position.

        Args:
            pose_goal: Goal pose passed in the message.
        """
        rospy.loginfo(f"{self.ns} : {parse_pose(pose_goal)}")
        self.pose_goal = pose_goal

    def move_towards_goal_pose(self) -> None:
        # Simulate movement towards goal pose
        self.pose = move_towards_pose(self.pose, self.pose_goal)

    def publish_current_pose(self) -> None:
        """Publish current pose."""
        rospy.logdebug(f"{self.pub_link}: Publishing: {parse_pose(self.pose)}")
        self.publisher.publish(self.pose)


def move_towards_pose(pose_start: Pose, pose_goal: Pose) -> Pose:
    """Simulates intermediate movement towards a goal pose from start pose."""
    step_x = 0.05
    step_y = 0.05

    delta_x = pose_goal.position.x - pose_start.position.x
    delta_y = pose_goal.position.y - pose_start.position.y

    sign_x = math.copysign(1, delta_x)
    sign_y = math.copysign(1, delta_y)

    new_x = pose_start.position.x + sign_x * min(step_x, abs(delta_x))
    new_y = pose_start.position.y + sign_y * min(step_y, abs(delta_y))

    return Pose(Point(new_x, new_y, 0), Quaternion(0, 0, 0, 1))


if __name__ == "__main__":
    Agent().start(30)
