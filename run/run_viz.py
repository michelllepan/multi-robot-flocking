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
    all_positions = np.vstack((
        boids_plotter.robot_positions,
        boids_plotter.target_positions))
    all_colors = (
        [ROBOT_COLOR] * num_robots +
        [TARGET_COLOR] * num_robots)
    scat = ax.scatter(
        x=all_positions[:, 0],
        y=all_positions[:, 1],
        c=all_colors)
    
    ax.set_xlim(-1, 7)
    ax.set_ylim(-1, 6)
    ax.add_patch(
        Rectangle(
            (0,10), 1100, 6, linestyle = 'dashed', facecolor = 'None', clip_on=False))

    robot_patch = Line2D([0], [0], marker="o", color=ROBOT_COLOR, linestyle="None", label="robot")
    target_patch = Line2D([0], [0], marker="o", color=TARGET_COLOR, linestyle="None", label="target")
    
    ax.legend(handles=[robot_patch, target_patch])

    def update(frame, scat, boids_plotter):
        scat.set_offsets(
            np.vstack((
                boids_plotter.robot_positions,
                boids_plotter.target_positions)))
        # print(boids_plotter.robot_positions)
        return scat,

    ani = animation.FuncAnimation(
        fig=fig,
        func=partial(update, scat=scat, boids_plotter=boids_plotter),
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