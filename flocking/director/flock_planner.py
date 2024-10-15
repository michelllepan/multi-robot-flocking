import time

import numpy as np
import redis

from .human_filter import HumanFilter
from flocking.boids import BoidsRunner
from flocking.utils import Goal, Pose, Humans

GOAL_TOLERANCE = 0.1

X_MAX = 6
Y_MAX = 4

class FlockPlanner:

    def __init__(self, robots, redis_config):
        super().__init__()
        self.robots = robots

        # redis setup
        self.redis_clients = {}
        self.redis_keys = {}

        for i in redis_config:
            host, port = redis_config[i]
            self.redis_clients[i] = redis.Redis(host, port)

        for r in self.robots:
            self.redis_keys[r] = {}
            self.redis_keys[r]["goal"] = "robot_" + str(r) + "::goal"
            self.redis_keys[r]["carrot"] = "robot_" + str(r) + "::carrot"
            self.redis_keys[r]["pose"] = "robot_" + str(r) + "::pose"
            self.redis_keys[r]["humans"] = "robot_" + str(r) + "::humans"
            self.redis_keys[r]["head"] = "robot_" + str(r) + "::head"
            self.redis_keys[r]["look"] = "robot_" + str(r) + "::look"

        self.filtered_human_key = "filtered_human"
        self.flock_state_key = "state"
        self.mode_key = "mode"

        # populate data
        self.flock_state = "STOP"
        self.mode = "DEFAULT"
        self.goals = {}
        self.carrots = {}
        self.looks = {}
        self.poses = {}
        self.humans = {}
        self.heads = {}
        self.read_redis()

        # Boids setup
        while len(self.poses) != len(self.robots):
            print("waiting for robot poses")
            time.sleep(0.5)
            self.read_redis()

        robot_starts = np.array([[self.poses[r].x, self.poses[r].y] for r in self.robots])
        self.boids_runner = BoidsRunner(
            num_robots=len(robots),
            robot_start_positions=robot_starts,
            step_scale=0.1,
            canvas_dims=(X_MAX, Y_MAX))

        # human setup
        self.human_filter = HumanFilter(
            robots=self.robots,
            x_min=0, x_max=X_MAX,
            y_min=0, y_max=Y_MAX)
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
        else:
            self.human = None

        # human_candidates = [h for humans in self.humans.values() for h in humans.coords]
        # if human_candidates:
        #     human = self.human_filter.apply(human_candidates, 1)
        #     if human:
        #         human_array = np.array(human).reshape((2,))
        #         self.boids_runner.move_human(human_array)
        #         self.human = human

        # update base targets
        self.boids_runner.update_targets(steps=2, mode=self.mode)
        carrots = self.boids_runner._get_carrots()
        for i in range(len(self.robots)):
            r = self.robots[i]
            t = self.boids_runner.target_positions[i]
            x = float(np.clip(t[0], 0, X_MAX))
            y = float(np.clip(t[1], 0, Y_MAX))
            self.goals[r] = Goal(x=x, y=y)

            # update carrots if robots are circling
            if self.mode == "CIRCLE":
                self.carrots[r] = Goal(x=float(carrots[i][0]), y=float(carrots[i][1]))
            else:
                self.carrots[r] = None

        # update head targets
        for r in self.robots:
            # head = self.heads[r]
            if self.human:
                look = np.atan2(self.human[1] - self.poses[r].y, self.human[0] - self.poses[r].x)
            else:
                # look = np.sin((time.time() + r) / 4) * 1.5
                look = self.poses[r].h
            self.looks[r] = look

        self.write_redis()

    def read_redis(self):
        for r in self.robots:
            head_string = self.redis_clients[r].get(self.redis_keys[r]["head"])
            if not head_string: continue
            head = eval(head_string)
            self.heads[r] = head

            pose_string = self.redis_clients[r].get(self.redis_keys[r]["pose"])
            if not pose_string: continue
            pose = Pose.from_string(pose_string)
            if pose is None: continue
            self.poses[r] = pose

            human_string = self.redis_clients[r].get(self.redis_keys[r]["humans"])
            if not human_string: continue
            humans = Humans.from_string(human_string)
            if humans is None: continue
            self.humans[r] = humans

        mode_string = self.redis_clients[0].get(self.mode_key)
        if mode_string is not None:
            self.mode = mode_string.decode("utf-8")

        flock_state_string = self.redis_clients[0].get(self.flock_state_key)
        if flock_state_string is not None:
            self.flock_state = flock_state_string.decode("utf-8")

    def write_redis(self):
        for r in self.robots:
            if r not in self.goals: continue
            self.redis_clients[r].set(self.redis_keys[r]["goal"], str(self.goals[r]))
            self.redis_clients[r].set(self.redis_keys[r]["look"], str(self.looks[r]))
            self.redis_clients[r].set(self.redis_keys[r]["carrot"], str(self.carrots[r]))
            self.redis_clients[r].set(self.filtered_human_key, str(self.human))
            self.redis_clients[r].set(self.flock_state_key, self.flock_state)
            self.redis_clients[r].set(self.mode_key, self.mode)