#!/usr/bin/env python3

import math
from logging import getLogger

import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion

from sadg_controller.comms import parse_pose

logger = getLogger(__name__)


class Agent:
    def __init__(self, ros_rate: int = 15) -> None:
        """Agent simulation.

        Args:
            ros_rate: Rate of agent simulation in Hz. Defaults to 30 Hz.

        """
        rospy.init_node("agent")
        self.ns = rospy.get_param("~agent_ns")
        self.uuid = rospy.get_param("~uuid")
        self.ros_rate = ros_rate

        self.sub_link_goal = f"/{self.ns}/goal"
        self.subscriber = rospy.Subscriber(self.sub_link_goal, Pose, self.callback)

        self.pub_link = f"/{self.ns}/current"
        self.publisher = rospy.Publisher(
            self.pub_link, Pose, queue_size=1000, latch=True
        )

        self.sub_link_initial = f"/{self.ns}/initial"
        self.subscriber_initial = rospy.Subscriber(
            self.sub_link_initial, Pose, self.callback_initial
        )

        self.pose = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))
        self.pose_goal = None

    def start(self) -> None:
        """Start agent simulation."""
        rate = rospy.Rate(self.ros_rate)
        while not rospy.is_shutdown():
            self.pose = self.move_towards_goal_pose()
            self.publish_current_pose()
            rate.sleep()

    def callback(self, pose_goal: Pose) -> None:
        """Callback for subscriber to goal position.

        Updates the agent's goal pose when a new goal pose
        is published by the controller.

        Args:
            pose_goal: Goal pose passed in the message.
        """
        rospy.logwarn(
            f"{self.sub_link_goal} : Received goal pose: {parse_pose(pose_goal)}"
        )
        self.pose_goal = pose_goal

    def callback_initial(self, pose_initial: Pose) -> None:
        """Callback for subscriber to initial position.

        Instantiates an agent to a new, initial pose.
        Only used at the start of a simulation to move
        agent to the starting position.

        Args:
            pose_initial: Initial pose passed in the message.
        """
        rospy.logwarn(
            f"{self.sub_link_initial}: Received initial pose: {parse_pose(pose_initial)}"
        )
        self.pose = pose_initial

    def move_towards_goal_pose(
        self, speed_x: float = 1.8, speed_y: float = 1.8
    ) -> Pose:
        """Simulates intermediate movement towards a goal pose from start pose.

        If no goal is specified, the current pose is simply returned.

        Args:
            speed_x: Speed in x-direction in m/s. Default to 0.8 m/s.
            speed_y: Speed in y-direction in m/s. Default to 0.8 m/s.
        """
        if self.pose_goal is None:
            return self.pose

        noise = np.random.normal(1.0, 0.3)  # mean = 1.0, std dev = 0.1

        step_x = noise * speed_x / self.ros_rate
        step_y = noise * speed_y / self.ros_rate

        delta_x = self.pose_goal.position.x - self.pose.position.x
        delta_y = self.pose_goal.position.y - self.pose.position.y

        sign_x = math.copysign(1, delta_x)
        sign_y = math.copysign(1, delta_y)

        new_x = self.pose.position.x + sign_x * min(step_x, abs(delta_x))
        new_y = self.pose.position.y + sign_y * min(step_y, abs(delta_y))

        return Pose(Point(new_x, new_y, 0), Quaternion(0, 0, 0, 1))

    def publish_current_pose(self) -> None:
        """Publish current pose.

        Publishes the current pose of this agent.
        """
        rospy.logdebug(f"{self.ns} : Current pose: {parse_pose(self.pose)}")
        self.publisher.publish(self.pose)


if __name__ == "__main__":
    Agent().start()
