import numpy as np
import redis
from typing import Sequence

from flocking.utils import Pose


REDIS_HOST = "localhost"
REDIS_PORT = "6379"

class HumanFilter:

    def __init__(self,
        robots: Sequence[int],
        x_min: float,
        x_max: float,
        y_min: float,
        y_max: float,
    ):
        super().__init__()
        self.redis_client = redis.Redis(host=REDIS_HOST, port=REDIS_PORT)
        self.robots = robots
        self.bounds_filter = lambda p: (p[0] > x_min and p[0] < x_max and
                                        p[1] > y_min and p[1] < y_max)

        x_center = (x_min + x_max) / 2
        y_center = (y_min + y_max) / 2
        self.dist_to_center = lambda p: np.linalg.norm([p[0] - x_center, 
                                                        p[1] - y_center])
        
    def robot_filter(self, p):
        for r in self.robots:
            pose_str = self.redis_client.get("robot_" + str(r) + "::pose")
            if pose_str is None: continue
            pose = Pose.from_string(pose_str)
            
            if np.linalg.norm([pose.x - p[0], pose.y - p[1]]) < 0.4: 
                return False
        return True

    def apply(self, candidates: list, num: int):
        candidates = filter(self.robot_filter, candidates)
        in_bounds = filter(self.bounds_filter, candidates)
        in_bounds = list(in_bounds)

        # temporary: choose human closest to center
        if len(in_bounds) <= 1:
            return in_bounds
        return min(in_bounds, key=self.dist_to_center)

