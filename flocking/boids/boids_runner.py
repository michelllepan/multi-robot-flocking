import time
from typing import Tuple

import numpy as np

from flocking.boids.boids_utils import *
from flocking.boids.vector_utils import clip_by_norm
from flocking.weight_modes import get_weight_mode


class BoidsRunner:

    def __init__(
        self,
        num_robots: int = 6,
        robot_start_positions: np.ndarray = None,
        step_scale: float = 1e-2,
        canvas_dims: Tuple[int] = (6, 5),
        publish_carrots: bool = False,
    ):
        self.num_robots = num_robots
        self.width, self.height = canvas_dims
        
        if robot_start_positions is None:
            self.robot_positions = np.random.rand(num_robots, 2) * canvas_dims
        else:
            self.robot_positions = robot_start_positions
        self.last_robot_positions = np.copy(self.robot_positions)
        self.target_positions = np.copy(self.robot_positions)

        self.human_position = np.random.rand(2) * canvas_dims
        self.last_human_position = np.copy(self.human_position)

        self.last_time = time.time()
        self.current_time = time.time()
        self.paused_time = 0.0
        
        self.carrot_positions = self._get_carrots()
        self.step_scale = step_scale

    def _get_other_positions_static(
        self,
        robot_id: int,
        robot_positions: np.ndarray,
        human_position: np.ndarray,
    ) -> np.ndarray:
        if robot_id == 0:
            other_robots = robot_positions[1:]
        elif robot_id == self.num_robots - 1:
            other_robots = robot_positions[:-1]
        else:
            other_robots = np.vstack((
                robot_positions[:robot_id],
                robot_positions[robot_id + 1:]
            ))
        return np.vstack((other_robots, human_position))

    def _get_other_positions(
        self,
        robot_id: int,
    ) -> np.ndarray:
        return self._get_other_positions_static(
            robot_id=robot_id,
            robot_positions=self.robot_positions,
            human_position=self.human_position,
        )
    
    def _get_last_other_positions(
        self,
        robot_id: int,
    ) -> np.ndarray:
        return self._get_other_positions_static(
            robot_id=robot_id,
            robot_positions=self.last_robot_positions,
            human_position=self.last_human_position,
        )
    
    def _get_carrots(self):
        carrots = []
        for robot_id in range(self.num_robots):
            pos = determine_carrot_position(
                robot_id=robot_id,
                region=(0, self.width, 0, self.height),
                timestamp=self.current_time - self.paused_time,
                num_robots=self.num_robots)
            carrots.append(pos)
        return np.vstack(carrots)
    
    def _get_sticks(self):
        return determine_stick_positions(
            region=(0, self.width, 0, self.height),
            timestamp=self.current_time - self.paused_time,
            robot_positions=self.robot_positions,
        )
    
    def _boids_static(
        self,
        robot_id: int,
        robot_positions: np.ndarray,
        human_position: np.ndarray,
        last_robot_positions: np.ndarray,
        last_human_position: np.ndarray,
        current_time: float,
        last_time: float,
        mode: str = "DEFAULT",
    ) -> np.ndarray:
        weights = get_weight_mode(mode)
        this_position = robot_positions[robot_id]
        other_positions = self._get_other_positions_static(
            robot_id=robot_id,
            robot_positions=robot_positions,
            human_position=human_position)
        last_other_positions = self._get_other_positions_static(
            robot_id=robot_id,
            robot_positions=last_robot_positions,
            human_position=last_human_position)
        time_delta = current_time - last_time

        cohesion_vec = compute_cohesion(
            this_position=this_position,
            other_positions=other_positions,
        ) * weights.cohesion

        separation_vec = compute_separation(
            this_position=this_position,
            other_positions=other_positions,
        ) * weights.separation

        alignment_vec = compute_alignment(
            other_positions=other_positions,
            last_other_positions=last_other_positions,
            time_delta=time_delta,
        ) * weights.alignment

        drive_at_human_vec = compute_drive_at_human(
            this_position=this_position,
            human_position=human_position,
        ) * weights.drive_at_human

        bounds_aversion_vec = compute_bounds_aversion(
            this_position=this_position,
            region=(0, self.width, 0, self.height),
        ) * weights.bounds_aversion

        goal_vec = compute_goal(
            this_position=this_position,
            robot_id=robot_id,
            region=(0, self.width, 0, self.height),
            timestamp=current_time,
            num_robots=self.num_robots,
        ) * weights.goal

        linear_vec = compute_linear(
            robot_id=robot_id,
            region=(0, self.width, 0, self.height),
            timestamp=current_time,
            robot_positions=robot_positions,
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

    def _boids(
        self,
        robot_id: int,
        mode: str = "DEFAULT",
    ) -> np.ndarray:
        return self._boids_static(
            robot_id=robot_id,
            robot_positions=self.robot_positions,
            human_position=self.human_position,
            last_robot_positions=self.last_robot_positions,
            last_human_position=self.last_human_position,
            current_time=self.current_time - self.paused_time,
            last_time=self.last_time - self.paused_time,
            mode=mode,
        )
    
    def move_human(
        self,
        human_position: np.ndarray,
    ):
        self.last_human_position = np.copy(self.human_position)
        self.human_position = human_position

    def move_robots(
        self,
        robot_positions: np.ndarray,
    ):
        self.last_robot_positions = np.copy(self.robot_positions)
        self.robot_positions = robot_positions
    
    def update_targets(
        self,
        steps: int = 1,
        mode: str = "DEFAULT",
        pause_time: bool = False,
    ):
        self.last_time = self.current_time
        self.current_time = time.time()
        if pause_time:
            print("pausing time")
            self.paused_time += self.current_time - self.last_time

        self.carrot_positions = self._get_carrots()

        positions = self.robot_positions
        last_positions = self.last_robot_positions
        
        current_time = self.current_time - self.paused_time
        last_time = self.last_time - self.paused_time

        for i in range(steps):
            targets = np.zeros_like(positions)
            for robot_id in range(self.num_robots):
                distance_vec = self._boids_static(
                    robot_id=robot_id,
                    robot_positions=positions,
                    human_position=self.human_position,
                    last_robot_positions=last_positions,
                    last_human_position=self.human_position,
                    current_time=current_time,
                    last_time=last_time,
                    mode=mode,
                )
                direction_vec = clip_by_norm(distance_vec, self.step_scale)
                targets[robot_id] = positions[robot_id] + direction_vec

            last_positions = positions
            positions = targets

            time_delta = current_time - last_time
            last_time, current_time = current_time, current_time + 0.1

        self.target_positions = targets