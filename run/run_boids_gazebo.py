from functools import partial
from multiprocessing import Process

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import rospy
from matplotlib.lines import Line2D

from flocking.boids import BoidsRunner
from flocking.ros.robot import Robot


MODE = "CIRCLE"
TARGET_COLOR = "mediumpurple"
ROBOT_COLOR = "steelblue"


def viz(boids_runner: BoidsRunner):
    fig, ax = plt.subplots()
    all_positions = np.vstack((
        boids_runner.robot_positions,
        boids_runner.target_positions))
    all_colors = (
        [ROBOT_COLOR] * boids_runner.num_robots +
        [TARGET_COLOR] * boids_runner.num_robots)
    scat = ax.scatter(
        x=all_positions[:, 0],
        y=all_positions[:, 1],
        c=all_colors)
    
    ax.set_xlim(0, boids_runner.width)
    ax.set_ylim(0, boids_runner.height)
    ax.set_title(f"current mode: {MODE}")

    robot_patch = Line2D([0], [0], marker="o", color=ROBOT_COLOR, linestyle="None", label="robot")
    target_patch = Line2D([0], [0], marker="o", color=TARGET_COLOR, linestyle="None", label="target")
    
    ax.legend(handles=[robot_patch, target_patch])

    def update(frame, scat, boids_runner):
        scat.set_offsets(
            np.vstack((
                boids_runner.robot_positions,
                boids_runner.target_positions)))
        return scat,

    ani = animation.FuncAnimation(
        fig=fig,
        func=partial(update, scat=scat, boids_runner=boids_runner),
        frames=200,
        interval=50,
        blit=True,
    )
    plt.show()


def main():
    rospy.init_node('boids_circle')
    robots = [Robot(robot_id=i) for i in range(3)]
    for r in robots:
        r.update_odom()

    robot_starts = np.array([[r.x, r.y] for r in robots])
    boids_runner = BoidsRunner(
        num_robots=3,
        robot_start_positions=robot_starts,
        step_scale=0.02)
    rate = rospy.Rate(10.0)

    boids_runner.update_targets(steps=100, mode=MODE)

    t = 0
    while not rospy.is_shutdown():
        print(f"t = {t}")
        if t % 3 == 0:
            boids_runner.update_targets(steps=20, mode=MODE)
            for i, r in enumerate(robots):
                r.set_goal(
                    x=boids_runner.target_positions[i, 0],
                    y=boids_runner.target_positions[i, 1])
                
        for r in robots:
            if not r.check_at_pos_goal():
                r.move_toward_goal()
            r.update_odom()
        
        robot_positions = np.array([[r.x, r.y] for r in robots])
        boids_runner.move_robots(robot_positions)

        t += 1
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')