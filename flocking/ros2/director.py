from functools import partial

import redis

from .robot import Robot
from .utils import Goal, Pose

REDIS_HOST = "localhost"
REDIS_PORT = "6379"

# class Director(Node):

#     def __init__(self, robots=(1,)):
#         super().__init__("director")
#         self.robots = robots

#         self.voltage = {}
#         self.goal_pubs = {}

#         for r in robots:
#             # publishers
#             self.goal_pubs[r] = self.create_publisher(
#                 Point, f"/robot_{r}/goal", 1)

#             # subscribers
#             self.create_subscription(
#                 BatteryState, f"/robot_{r}/battery",
#                 partial(self.battery_callback, robot_id=r), 1)

#         # TODO: adjust rate
#         self.timer = self.create_timer(0.2, self.step_flocking)

#     def battery_callback(self, msg: BatteryState, robot_id: int):
#         self.voltage[robot_id] = msg.voltage

#     def step_flocking(self):
#         for r in self.robots:
#             goal = Point()
#             goal.x = 2.0
#             goal.y = 1.0
#             self.goal_pubs[r].publish(goal)


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

    def step_flocking(self):
        for r in self.robots:
            goal = Goal(x=0.0, y=0.0)
            self.redis_client.set(self.goal_keys[r], str(goal))