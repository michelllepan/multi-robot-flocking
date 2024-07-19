import argparse
from functools import partial

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import rospy
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle

from flocking.ros.subscribers import *

TARGET_COLOR = "mediumpurple"
ROBOT_COLOR = "steelblue"
CARROT_COLOR = "#ffc2b0"


class BoidsPlotter:

    def __init__(self, num_robots: int):
        rospy.init_node('boids_plotter')
        self.num_robots = num_robots
        self.robot_subs = [GroundTruthSubscriber(i) for i in range(num_robots)]
        self.target_subs = [TargetSubscriber(i) for i in range(num_robots)]
        self.carrot_subs = [CarrotSubscriber(i) for i in range(num_robots)]

    @property
    def robot_positions(self):
        return np.array([
            [s.position.x, s.position.y] if s.position else [0, 0]
            for s in self.robot_subs])

    @property
    def target_positions(self):
        return np.array([
            [s.data.x, s.data.y] if s.data else [0, 0]
            for s in self.target_subs])
    
    @property
    def carrot_positions(self):
        return np.array([
            [s.data.x, s.data.y] if s.data else [0, 0]
            for s in self.carrot_subs])



def main(num_robots: int = 3):
    boids_plotter = BoidsPlotter(num_robots)

    fig, ax = plt.subplots()

    scats = []
    for i in range(num_robots):
        scats.append(ax.scatter(
            x=boids_plotter.robot_positions[i][0],
            y=boids_plotter.robot_positions[i][1],
            c=ROBOT_COLOR,
            marker=f"${i}$"))
        scats.append(ax.scatter(
            x=boids_plotter.target_positions[i][0],
            y=boids_plotter.target_positions[i][1],
            c=TARGET_COLOR,
            marker=f"${i}$"))
        scats.append(ax.scatter(
            x=boids_plotter.carrot_positions[i][0],
            y=boids_plotter.carrot_positions[i][1],
            c=CARROT_COLOR,
            marker=f"${i}$"))
    
    ax.set_xlim(-1, 16)
    ax.set_ylim(-1, 13)

    robot_patch = Line2D([0], [0], marker="o", color=ROBOT_COLOR, linestyle="None", label="robot")
    target_patch = Line2D([0], [0], marker="o", color=TARGET_COLOR, linestyle="None", label="target")
    
    ax.legend(handles=[robot_patch, target_patch])

    def update(frame, scats, boids_plotter):
        for i in range(boids_plotter.num_robots):
            scats[i * 3].set_offsets(boids_plotter.robot_positions[i])
            scats[i * 3 + 1].set_offsets(boids_plotter.target_positions[i])
            scats[i * 3 + 2].set_offsets(boids_plotter.carrot_positions[i])
        return scats

    ani = animation.FuncAnimation(
        fig=fig,
        func=partial(update, scats=scats, boids_plotter=boids_plotter),
        frames=200,
        interval=50,
        blit=True,
    )
    plt.show()


if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument("--num_robots", "-n", type=int, default=3)
        args = parser.parse_args()
        main(num_robots=args.num_robots)
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')