import argparse
from functools import partial

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D

from flocking.boids import BoidsRunner


def main(
    mode: str,
    draw_carrots: bool,
    control_human: bool,
):
    fig, ax = plt.subplots()

    boids_runner = BoidsRunner(num_robots=3)
    robot_positions = boids_runner.robot_positions
    human_position = boids_runner.human_position
    carrot_positions = boids_runner.carrot_positions

    all_positions = np.vstack((carrot_positions, human_position, robot_positions))
    all_colors = (
        ["#ffc2b0" if draw_carrots else "white"] * boids_runner.num_robots +
        ["seagreen"] +
        ["steelblue"] * boids_runner.num_robots
    )
    scat = ax.scatter(
        x=all_positions[:, 0],
        y=all_positions[:, 1],
        c=all_colors)
    
    def handle_mouse(event):
        if event.xdata and event.ydata:
            boids_runner.move_human(np.array([event.xdata, event.ydata]))

    ax.set_xlim(0, boids_runner.width)
    ax.set_ylim(0, boids_runner.height)
    ax.set_title(f"current mode: {mode}")

    robot_patch = Line2D([0], [0], marker="o", color="steelblue", linestyle="None", label="robot")
    human_patch = Line2D([0], [0], marker="o", color="seagreen", linestyle="None", label="human")
    carrot_patch = Line2D([0], [0], marker="o", color="#ffc2b0", linestyle="None", label="carrot")
    
    if draw_carrots:
        ax.legend(handles=[robot_patch, human_patch, carrot_patch])
    else:
        ax.legend(handles=[robot_patch, human_patch])

    if control_human:
        plt.connect("motion_notify_event", handle_mouse)

    def update(frame, scat, boids_runner):
        boids_runner.update(mode=mode)
        scat.set_offsets(
            np.vstack((
                boids_runner.carrot_positions,
                boids_runner.human_position,
                boids_runner.robot_positions)))
        return scat,

    ani = animation.FuncAnimation(
        fig=fig,
        func=partial(update, scat=scat, boids_runner=boids_runner),
        frames=200,
        interval=50,
        blit=True,
    )
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", "-m", default="DEFAULT")
    parser.add_argument("--carrots", "-c", action="store_true")
    parser.add_argument("--human", action="store_true")
    args = parser.parse_args()

    main(
        mode=args.mode,
        draw_carrots=args.carrots,
        control_human=args.human,
    )