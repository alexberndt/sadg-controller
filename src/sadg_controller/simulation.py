#!/usr/bin/env python3

import matplotlib.pyplot as plt
import rospy


class Simulation:
    def __init__(self) -> None:

        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        self.ax.grid(True)
        self.ax.set_xlim((-80, 80))
        self.ax.set_ylim((-80, 80))
        self.ax.set_aspect("equal")

    def start(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

            rate.sleep()


if __name__ == "__main__":
    Simulation().start()
