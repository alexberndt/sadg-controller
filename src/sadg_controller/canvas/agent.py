#!/usr/bin/env python3

from typing import List

import matplotlib.colors as clr
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion


class Agent:
    def __init__(self, ns: str, ax: plt.Axes, color: str) -> None:

        self.ns = ns
        self.ax = ax
        self.color = clr.to_hex(color)

        self.pose = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))
        (self.position_current,) = self.ax.plot(
            get_x_points(self.pose.position.x),
            get_y_points(self.pose.position.y),
            self.color,
        )
        # self.position_goal, = self.ax.plot(self.pose.position.x, self.pose.position.y, 'g.-')

        self.sub_link = f"/{self.ns}/current"
        self.subscriber = rospy.Subscriber(self.sub_link, Pose, self.callback)

        # self.sub_link_goal = f"/{self.ns}/goal"
        # self.subscriber_goal = rospy.Subscriber(self.sub_link_goal, Pose, self.callback_goal)

        rospy.loginfo(f"Initialized Agent subscriber: {self.sub_link}")

    def callback(self, pose: Pose) -> None:
        rospy.loginfo("Callback pose")
        self.pose = pose
        self.update_plot()

    # def callback_goal(self, pose: Pose) -> None:
    #     rospy.loginfo("Callback goal pose")
    #     self.pose_goal = pose
    #     self.update_goal_plot()

    def update_plot(self) -> None:

        x = self.pose.position.x
        y = self.pose.position.y

        x_points = get_x_points(x)
        y_points = get_y_points(y)

        self.position_current.set_xdata(x_points)
        self.position_current.set_ydata(y_points)

    # def update_goal_plot(self) -> None:
    #     self.position_goal.set_xdata(self.pose_goal.position.x)
    #     self.position_goal.set_ydata(self.pose_goal.position.y)


def get_x_points(x: float, h: float = 0.7) -> List[float]:
    return [x - h, x - h, x + h, x + h, x - h]


def get_y_points(y: float, h: float = 0.7) -> List[float]:
    return [y - h, y + h, y + h, y - h, y - h]
