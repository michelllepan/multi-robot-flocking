from functools import partial

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import rospy
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle

from flocking.ros.subscribers import GroundTruthSubscriber, TargetSubscriber

TARGET_COLOR = "mediumpurple"
ROBOT_COLOR = "steelblue"


class BoidsPlotter:

    def __init__(self, num_robots: int):
        rospy.init_node('boids_plotter')
        self.num_robots = num_robots
        self.robot_subs = [GroundTruthSubscriber(i) for i in range(num_robots)]
        self.target_subs = [TargetSubscriber(i) for i in range(num_robots)]

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
    
    ax.set_xlim(-1, 7)
    ax.set_ylim(-1, 6)

    robot_patch = Line2D([0], [0], marker="o", color=ROBOT_COLOR, linestyle="None", label="robot")
    target_patch = Line2D([0], [0], marker="o", color=TARGET_COLOR, linestyle="None", label="target")
    
    ax.legend(handles=[robot_patch, target_patch])

    def update(frame, scats, boids_plotter):
        for i in range(boids_plotter.num_robots):
            scats[i * 2].set_offsets(boids_plotter.robot_positions[i])
            scats[i * 2 + 1].set_offsets(boids_plotter.target_positions[i])
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
        main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')