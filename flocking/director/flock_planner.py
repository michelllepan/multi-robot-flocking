import time

import numpy as np
import redis

from .human_filter import HumanFilter
from flocking.boids import BoidsRunner
from flocking.utils import Goal, Pose, Humans

GOAL_TOLERANCE = 0.1

REDIS_HOST = "localhost"
# REDIS_HOST = "10.5.90.8"
REDIS_PORT = "6379"

X_MAX = 4
Y_MAX = 4

class FlockPlanner:

    def __init__(self, robots=(1,)):
        super().__init__()
        self.robots = robots

        # redis setup
        self.redis_client = redis.Redis(host=REDIS_HOST, port=REDIS_PORT)
        self.redis_keys = {}

        for r in self.robots:
            self.redis_keys[r] = {}
            self.redis_keys[r]["goal"] = "robot_" + str(r) + "::goal"
            self.redis_keys[r]["pose"] = "robot_" + str(r) + "::pose"
            self.redis_keys[r]["humans"] = "robot_" + str(r) + "::humans"
            self.redis_keys[r]["head"] = "robot_" + str(r) + "::head"

        self.filtered_human_key = "filtered_human"

        # populate data
        self.goals = {}
        self.looks = {}
        self.poses = {}
        self.humans = {}
        self.heads = {}
        self.read_redis()

        # Boids setup
        robot_starts = np.array([[self.poses[r].x, self.poses[r].y] for r in self.robots])
        self.boids_runner = BoidsRunner(
            num_robots=len(robots),
            robot_start_positions=robot_starts,
            step_scale=0.1,
            canvas_dims=(X_MAX, Y_MAX))

        # human setup
        self.human_filter = HumanFilter(x_min=-1, x_max=X_MAX, y_min=-1, y_max=Y_MAX)
        self.human = None

    def step_flocking(self):
        self.read_redis()
        robot_positions = np.array([[self.poses[r].x, self.poses[r].y] for r in self.robots])
        self.boids_runner.move_robots(robot_positions)

        # update human location
        human_candidates = [h for humans in self.humans.values() for h in humans.coords]
        if human_candidates:
            self.human = self.human_filter.apply(human_candidates, 1)
            if self.human:
                human_array = np.array(self.human).reshape((2,))
                self.boids_runner.move_human(human_array)
        else:
            self.human = None

        # update base targets
        self.boids_runner.update_targets(steps=20, mode="FOLLOW")
        for i in range(len(self.robots)):
            r = self.robots[i]
            t = self.boids_runner.target_positions[i]
            self.goals[r] = Goal(x=float(t[0]), y=float(t[1]))

        # update head targets
        for r in self.robots:
            head = self.heads[r]
            if self.human:
                look = np.atan2(self.human[1] - self.pose.y, self.human[0] - self.pose.x)
            else:
                look = np.sin((time.time() + r) / 4) * 1.5
            self.looks[r] = look

        self.write_redis()

    def read_redis(self):
        for r in self.robots:
            head_string = self.redis_client.get(self.redis_keys[r]["head"])
            if not head_string: continue
            head = eval(head)
            self.heads[r] = head

            pose_string = self.redis_client.get(self.redis_keys[r]["pose"])
            if not pose_string: continue
            pose = Pose.from_string(pose_string)
            if pose is None: continue
            self.poses[r] = pose

            human_string = self.redis_client.get(self.redis_keys[r]["humans"])
            if not human_string: continue
            humans = Humans.from_string(human_string)
            if humans is None: continue
            self.humans[r] = humans

    def write_redis(self):
        for r in self.robots:
            if r not in self.goals: continue
            self.redis_client.set(self.redis_keys[r]["goal"], str(self.goals[r]))
            self.redis_client.set(self.redis_keys[r]["look"], str(self.looks[r]))

        self.redis_client.set(self.filtered_human_key, str(self.human))