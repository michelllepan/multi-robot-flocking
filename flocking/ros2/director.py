from functools import partial

import redis

from .robot import Robot
from .utils import Goal, Pose

GOAL_TOLERANCE = 0.1

REDIS_HOST = "localhost"
REDIS_PORT = "6379"


class Director:

    def __init__(self, robots=(1,)):
        super().__init__()
        self.robots = robots
        self.redis_client = redis.Redis(host=REDIS_HOST, port=REDIS_PORT)

        self.goal_keys = {}
        self.pose_keys = {}

        self.goals = {}
        self.poses = {}

        for r in self.robots:
            self.goal_keys[r] = "robot_" + str(r) + "::goal"
            self.pose_keys[r] = "robot_" + str(r) + "::pose"

        self.goals = [
            Goal(x=0.0, y=1.0),
            Goal(x=1.0, y=1.0),
            Goal(x=1.0, y=0.0),
            Goal(x=0.0, y=0.0)
        ]
        self.i = 0

    def step_flocking(self):
        for r in self.robots:
            pose_string = self.redis_client.get(self.pose_keys[r])
            pose = Pose.from_string(pose_string)
            if self.i >= len(self.goals): return

            goal = self.goals[self.i]
            self.redis_client.set(self.goal_keys[r], str(goal))

            if self.pose_equals_goal(pose, goal):
                self.i += 1

    def pose_equals_goal(self, pose: Pose, goal: Goal):
        if pose is None or goal is None:
            return False
        return (abs(pose.x - goal.x) < GOAL_TOLERANCE and 
                abs(pose.y - goal.y) < GOAL_TOLERANCE)