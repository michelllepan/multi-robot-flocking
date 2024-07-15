import argparse
import time
from functools import partial
from typing import Tuple

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D

from flocking.boids.boids_utils import *
from flocking.boids.vector_utils import clip_by_norm
from flocking.weight_modes import get_weight_mode


class BoidsRunner:

    def __init__(
        self,
        num_robots: int = 6,
        canvas_dims: Tuple[int] = (6, 5),
    ):
        self.num_robots = num_robots
        self.width, self.height = canvas_dims
        
        self.robot_positions = np.random.rand(num_robots, 2) * canvas_dims
        self.last_robot_positions = np.copy(self.robot_positions)

        self.human_position = np.random.rand(2) * canvas_dims
        self.last_human_position = np.copy(self.human_position)

        self.last_time = time.time()
        self.current_time = time.time()

        self.carrot_positions = self._get_carrots()

    def _get_other_positions(
        self,
        robot_id: int,
    ) -> np.ndarray:
        if robot_id == 0:
            other_robots = self.robot_positions[1:]
        elif robot_id == self.num_robots - 1:
            other_robots = self.robot_positions[:-1]
        else:
            other_robots = np.vstack((
                self.robot_positions[:robot_id],
                self.robot_positions[robot_id + 1:]
            ))
        return np.vstack((other_robots, self.human_position))
    
    def _get_last_other_positions(
        self,
        robot_id: int,
    ) -> np.ndarray:
        if robot_id == 0:
            other_robots = self.last_robot_positions[1:]
        elif robot_id == self.num_robots - 1:
            other_robots = self.last_robot_positions[:-1]
        else:
            other_robots = np.vstack((
                self.last_robot_positions[:robot_id],
                self.last_robot_positions[robot_id + 1:]
            ))
        return np.vstack((other_robots, self.last_human_position))
    
    def _get_carrots(self):
        carrots = []
        for robot_id in range(self.num_robots):
            carrots.append(
                determine_carrot_position(
                    robot_id=robot_id,
                    region=(0, self.width, 0, self.height),
                    timestamp=self.current_time,
                    num_robots=self.num_robots))
        return np.vstack(carrots)

    def _boids(
        self,
        robot_id: int,
        mode: str = "DEFAULT",
    ) -> np.ndarray:
        weights = get_weight_mode(mode)
        this_position = self.robot_positions[robot_id]

        cohesion_vec = compute_cohesion(
            this_position=this_position,
            other_positions=self._get_other_positions(robot_id),
        ) * weights.cohesion

        separation_vec = compute_separation(
            this_position=this_position,
            other_positions=self._get_other_positions(robot_id),
        ) * weights.separation

        alignment_vec = compute_alignment(
            other_positions=self._get_other_positions(robot_id),
            last_other_positions=self._get_last_other_positions(robot_id),
            time_delta=(self.current_time - self.last_time),
        ) * weights.alignment

        drive_at_human_vec = compute_drive_at_human(
            this_position=this_position,
            human_position=self.human_position,
        ) * weights.drive_at_human

        bounds_aversion_vec = compute_bounds_aversion(
            this_position=this_position,
            region=(0, self.width, 0, self.height),
            margin=0.5,
        ) * weights.bounds_aversion

        goal_vec = compute_goal(
            this_position=this_position,
            robot_id=robot_id,
            region=(0, self.width, 0, self.height),
            timestamp=self.current_time,
            num_robots=self.num_robots,
        ) * weights.goal

        linear_vec = compute_linear(
            robot_id=robot_id,
            region=(0, self.width, 0, self.height),
            timestamp=self.current_time,
            robot_positions=self.robot_positions,
        ) * weights.linear

        distance_vec = np.sum((
            cohesion_vec,
            separation_vec,
            alignment_vec,
            drive_at_human_vec,
            bounds_aversion_vec,
            goal_vec,
            linear_vec,
        ), axis=0)
        return distance_vec
    
    def move_human(
        self,
        human_position: np.ndarray,
    ):
        self.last_human_position = np.copy(self.human_position)
        self.human_position = human_position
    
    def update(
        self,
        mode: str = "DEFAULT",
    ):
        self.last_time = self.current_time
        self.current_time = time.time()

        self.carrot_positions = self._get_carrots()
        self.last_robot_positions = np.copy(self.robot_positions)

        for robot_id in range(self.num_robots):
            distance_vec = self._boids(robot_id, mode)
            direction_vec = clip_by_norm(distance_vec, 1e-2)
            self.robot_positions[robot_id] += direction_vec

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