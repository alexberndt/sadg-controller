#!/usr/bin/env python3
# sadg-controller
# Copyright (c) 2023 Alexander Berndt, Robert Bosch GmbH
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as published
# by the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Affero General Public License for more details.
#
# You should have received a copy of the GNU Affero General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

from typing import List

import matplotlib.colors as clr
import matplotlib.pyplot as plt
from geometry_msgs.msg import Point, Pose, Quaternion
from rclpy.node import Node


class Agent:
    def __init__(self, node: Node, ns: str, ax: plt.Axes, color: str) -> None:

        self.node = node
        self.ns = ns
        self.ax = ax
        self.color = clr.to_hex(color)

        self.pose = Pose(
            position=Point(x=0.0, y=0.0, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        (self.position_current,) = self.ax.plot(
            get_x_points(self.pose.position.x),
            get_y_points(self.pose.position.y),
            self.color,
        )
        # self.position_goal, = self.ax.plot(self.pose.position.x, self.pose.position.y, 'g.-')

        self.sub_link = f"/{self.ns}/current"
        self.subscriber = node.create_subscription(
            Pose, self.sub_link, self.callback, 10
        )

        # self.sub_link_goal = f"/{self.ns}/goal"
        # self.subscriber_goal = rospy.Subscriber(self.sub_link_goal, Pose, self.callback_goal)

        node.get_logger().info(f"Initialized Agent subscriber: {self.sub_link}")

    def callback(self, pose: Pose) -> None:
        self.node.get_logger().debug("Callback pose")
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
