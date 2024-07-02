import time
from functools import partial
from typing import Tuple

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np

from flocking.utils.boid_utils import *
from flocking.utils.vector_utils import clip_by_norm
from flocking.weight_modes import get_weight_mode


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

    def _boid(
        self,
        robot_id: int,
        mode: str = "DEFAULT",
    ) -> np.ndarray:
        weights = get_weight_mode(mode)
        # from flocking.weight_modes.modes import WeightMode
        # weights = WeightMode(
        #     goal             =0.0,
        #     bounds_aversion  =0.0,
        #     cohesion         =0.0,
        #     separation       =1.0,
        #     alignment        =1.0,
        #     drive_at_human   =0.1,
        #     linear           =0.0,
        # )
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
    
    def update(self, mode: str = "DEFAULT"):
        self.last_time = self.current_time
        self.current_time = time.time()

        self.last_robot_positions = np.copy(self.robot_positions)

        for robot_id in range(self.num_robots):
            distance_vec = self._boid(robot_id, mode)
            direction_vec = clip_by_norm(distance_vec, 1.0)
            self.robot_positions[robot_id] += direction_vec

def main():
    fig, ax = plt.subplots()

    boid_runner = BoidRunner()
    robot_positions = boid_runner.robot_positions
    scat = ax.scatter(x=robot_positions[:, 0], y=robot_positions[:, 1])

    ax.set_xlim(0, boid_runner.width)
    ax.set_ylim(0, boid_runner.height)

    print(f"HUMAN is at {boid_runner.human_position}")

    def update(frame, scat, boid_runner):
        boid_runner.update(mode="CIRCLE")
        scat.set_offsets(boid_runner.robot_positions)
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
    main()