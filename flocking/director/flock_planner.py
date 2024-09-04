import numpy as np
import redis

from flocking.boids import BoidsRunner
from flocking.utils import Goal, Pose

GOAL_TOLERANCE = 0.1

REDIS_HOST = "localhost"
REDIS_PORT = "6379"


class FlockPlanner:

    def __init__(self, robots=(1,)):
        super().__init__()
        self.robots = robots

        # redis setup
        self.redis_client = redis.Redis(host=REDIS_HOST, port=REDIS_PORT)
        self.goal_keys = {}
        self.pose_keys = {}

        for r in self.robots:
            self.goal_keys[r] = "robot_" + str(r) + "::goal"
            self.pose_keys[r] = "robot_" + str(r) + "::pose"

        # populate data
        self.goals = {}
        self.poses = {}
        self.sync_redis()

        # Boids setup
        robot_starts = np.array([[self.poses[r].x, self.poses[r].y] for r in self.robots])
        self.boids_runner = BoidsRunner(
            num_robots=len(robots),
            robot_start_positions=robot_starts,
            step_scale=0.1,
            canvas_dims=(4,4))

    def step_flocking(self):
        # update robot positions
        self.sync_redis()
        robot_positions = np.array([[self.poses[r].x, self.poses[r].y] for r in self.robots])
        self.boids_runner.move_robots(robot_positions)

        # update targets
        self.boids_runner.update_targets(steps=20, mode="LINEAR_TRACKS")
        print(self.boids_runner.target_positions)
        for i in range(len(self.robots)):
            r = self.robots[i]
            t = self.boids_runner.target_positions[i]
            self.goals[r] = Goal(x=float(t[0]), y=float(t[1]))

        
    def sync_redis(self):
        for r in self.robots:
            pose_string = self.redis_client.get(self.pose_keys[r])
            if not pose_string: continue

            pose = Pose.from_string(pose_string)
            if pose is None: continue
            self.poses[r] = pose

            if r not in self.goals: continue
            self.redis_client.set(self.goal_keys[r], str(self.goals[r]))

            # without robots
            # if r in self.goals:
            #     goal = self.goals[r]
            # else:
            #     goal = Goal(x=float(r), y=0.0)

            # self.goals[r] = goal
            # self.redis_client.set(self.goal_keys[r], str(goal))

            # pose = Pose(x=float(goal.x), y=float(goal.y), h=0.0)

            # self.poses[r] = goal
            # self.redis_client.set(self.pose_keys[r], str(pose))
