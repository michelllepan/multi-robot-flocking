import argparse
import time
from functools import partial
from typing import Tuple

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np

from flocking.utils.boid_utils import *
from flocking.utils.vector_utils import clip_by_norm
from flocking.weight_modes import get_weight_mode


ROBOT_MARGIN = 5

class BoidRunner:

    def __init__(
        self,
        num_robots: int = 6,
        canvas_dims: Tuple[int] = (480, 360),
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

    def _boid(
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
            margin=20,
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
    
    def _maybe_move_robot(
        self,
        robot_id: int,
        direction_vec: np.ndarray,
    ) -> bool:
        new_position = self.robot_positions[robot_id] + direction_vec
        for other_pos in self._get_other_positions(robot_id):
            if np.linalg.norm(new_position - other_pos) < ROBOT_MARGIN:
                return False
        self.robot_positions[robot_id] = new_position
        return True
    
    def update(
        self,
        mode: str = "DEFAULT",
    ):
        self.last_time = self.current_time
        self.current_time = time.time()

        self.last_robot_positions = np.copy(self.robot_positions)

        for robot_id in range(self.num_robots):
            distance_vec = self._boid(robot_id, mode)
            direction_vec = clip_by_norm(distance_vec, 1.0)
            self._maybe_move_robot(robot_id, direction_vec)

        self.carrot_positions = self._get_carrots()


def main(mode: str):
    fig, ax = plt.subplots()

    boid_runner = BoidRunner()
    robot_positions = boid_runner.robot_positions
    human_position = boid_runner.human_position
    carrot_positions = boid_runner.carrot_positions

    all_positions = np.vstack((robot_positions, human_position, carrot_positions))
    all_colors = ["steelblue"] * boid_runner.num_robots + ["seagreen"] + ["coral"] * boid_runner.num_robots
    scat = ax.scatter(
        x=all_positions[:, 0],
        y=all_positions[:, 1],
        c=all_colors)

    ax.set_xlim(0, boid_runner.width)
    ax.set_ylim(0, boid_runner.height)

    print(f"HUMAN is at {human_position}")

    def update(frame, scat, boid_runner):
        boid_runner.update(mode=mode)
        scat.set_offsets(
            np.vstack((
                boid_runner.robot_positions,
                boid_runner.human_position,
                boid_runner.carrot_positions)))
        return scat,

    ani = animation.FuncAnimation(
        fig=fig,
        func=partial(update, scat=scat, boid_runner=boid_runner),
        frames=200,
        interval=50,
        blit=True,
    )
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", "-m", default="DEFAULT")
    args = parser.parse_args()
    main(mode=args.mode)