import math
from typing import Sequence, Tuple

import numpy as np

from flocking.utils.vector_utils import normalize, clip_by_norm


# period for carrot function
CARROT_PERIOD = 50 # seconds

# cap goal influence
GOAL_CAP = 2

# cap linear influence
LINEAR_CAP = 2

# period for linear function
LINEAR_PERIOD = 37 # seconds

# how close robots have to be before separation becomes a concern
SEPARATION_MIN_DISTANCE = 50


def compute_cohesion(
    this_position: np.ndarray,
    other_positions: Sequence[np.ndarray],
) -> np.ndarray:
    """
    Compute Cohesion - Boids want to stay close.
    """
    assert this_position.shape == (2,)

    other_positions = np.array(other_positions)
    assert other_positions.ndim == 2 and other_positions.shape[1] == 2

    average_position = other_positions.mean(axis=0)
    cohesion_vec = average_position - this_position

    cohesion_vec = normalize(cohesion_vec)
    return cohesion_vec


def compute_separation(
    this_position : np.ndarray,
    other_positions: Sequence[np.ndarray],
) -> np.ndarray:
    """
    Compute Separation - Boids want to stay apart.
    """
    assert this_position.shape == (2,)

    other_positions = np.array(other_positions)
    assert other_positions.ndim == 2 and other_positions.shape[1] == 2

    separation_vec = np.zeros((2,), dtype=float)

    distance_vecs = other_positions - this_position
    for i in range(distance_vecs.shape[0]):
        distance_vec = distance_vecs[i]
        separation = np.linalg.norm(distance_vec) - SEPARATION_MIN_DISTANCE
        if separation < 0:
            separation_vec += -distance_vec * separation

    separation_vec = normalize(separation_vec)
    return separation_vec


def compute_alignment(
    other_positions: Sequence[np.ndarray],
    last_other_positions: Sequence[np.ndarray],
    time_delta: float,
) -> np.ndarray:
    """
    Compute Alignment - Boids want to move together.
    """
    assert len(other_positions) == len(last_other_positions)

    other_positions = np.array(other_positions)
    last_other_positions = np.array(last_other_positions)
    assert other_positions.ndim == 2 and other_positions.shape[1] == 2
    assert other_positions.shape == last_other_positions.shape

    other_velocities = (other_positions - last_other_positions) / time_delta
    average_velocity = other_velocities.mean(axis=0)

    alignment_vec = normalize(average_velocity)
    return alignment_vec


def compute_drive_at_human(
    this_position: np.ndarray,
    human_position: np.ndarray,
) -> np.ndarray:
    assert this_position.shape == (2,)
    assert human_position.shape == (2,)

    drive_at_vec = human_position - this_position
    drive_at_vec = normalize(drive_at_vec)
    return drive_at_vec


def compute_bounds_aversion(
    this_position: np.ndarray,
    region: Tuple[float],
    margin: float,
) -> np.ndarray:
    assert this_position.shape == (2,)
    rx, ry = this_position[0], this_position[1]

    x_min, x_max, y_min, y_max = region
    assert x_min < x_max and y_min < y_max

    # make aversion relative to how far in the margin the robot is
    if rx < x_min + margin:
        vx = (x_min + margin) - rx
    elif rx > x_max - margin:
        vx = (x_max - margin) - rx
    else:
        vx = 0

    if ry < y_min + margin:
        vy = y_min + margin - ry
    elif ry > y_max - margin:
        vy = (y_max - margin) - ry
    else:
        vy = 0

    aversion_vec = np.array([vx, vy], dtype=float)
    return aversion_vec


def determine_carrot_position(
    robot_id: int,
    region: Tuple[float],
    timestamp: float,
    num_robots: int,
) -> np.ndarray:
    """
    Computes position of a carrot moving on a circle.
    """
    x_min, x_max, y_min, y_max = region
    assert x_min < x_max and y_min < y_max

    center_x = (x_min + x_max) / 2
    center_y = (y_min + y_max) / 2
    radius = min(x_max - x_min, y_max - y_min) * 0.9

    # distributes the robots equally on a single track
    timestamp += robot_id / num_robots * CARROT_PERIOD

    pos_radians = 2 * math.pi * (timestamp % CARROT_PERIOD) / CARROT_PERIOD
    pos_x = radius * math.cos(pos_radians) + center_x
    pos_y = radius * math.sin(pos_radians) + center_y

    carrot_position = np.array([pos_x, pos_y])
    return carrot_position


def compute_goal(
    this_position: np.ndarray,
    robot_id: int,
    region: Tuple[float],
    timestamp: float,
    num_robots: int,
) -> np.ndarray:
    assert robot_id < num_robots

    carrot_position = determine_carrot_position(
        robot_id=robot_id,
        region=region,
        timestamp=timestamp,
        num_robots=num_robots,
    )
    distance_vec = carrot_position - this_position
    goal_vec = clip_by_norm(distance_vec, GOAL_CAP)
    return goal_vec


def compute_linear(
    robot_id: int,
    region: Tuple[float],
    timestamp: float,
    robot_positions: Sequence[np.ndarray],
) -> np.ndarray:
    assert robot_id < len(robot_positions)

    x_min, x_max, y_min, y_max = region
    width, height = x_max - x_min, y_max - y_min
    assert width > 0 and height > 0

    robot_positions = np.array(robot_positions)
    assert(robot_positions.ndim == 2 and robot_positions.shape[1] == 2)

    radians = 2 * math.pi * (timestamp % LINEAR_PERIOD) / LINEAR_PERIOD
    sorted_indices = np.argsort(robot_positions, axis=0)
    num_robots = len(robot_positions)

    if width > height:
        robot_lane = sorted_indices[robot_id][0] # assign lane from order on x-axis
        lane_x = x_min + (width / num_robots * robot_lane)
        center_y = (y_min + y_max) / 2
        goal_x = (0.75 * np.cos(radians)) + lane_x + 2
        goal_y = (np.sin(radians) * height / 2) + center_y

    else:
        robot_lane = sorted_indices[robot_id][1] # assign lane from order on y-axis
        lane_y = y_min + (height / num_robots * robot_lane)
        center_x = (x_min + x_max) / 2
        goal_y = (0.75 * np.cos(radians)) + lane_y + 2
        goal_x = (np.sin(radians) * width / 2) + center_x

    goal_position = np.array([goal_x, goal_y])
    this_position = robot_positions[robot_id]
    distance_vec = goal_position - this_position
    return clip_by_norm(distance_vec, LINEAR_CAP)
