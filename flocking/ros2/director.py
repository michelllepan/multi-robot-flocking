from functools import partial

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from sensor_msgs.msg import BatteryState, LaserScan

from .robot import Robot


class Director(Node):

    def __init__(self, robots=(1,)):
        super().__init__("director")
        self.robots = robots

        self.voltage = {}
        self.goal_pubs = {}

        for r in robots:
            # publishers
            self.goal_pubs[r] = self.create_publisher(
                Point, f"/robot_{r}/goal", 1)

            # subscribers
            self.create_subscription(
                BatteryState, f"/robot_{r}/battery",
                partial(self.battery_callback, robot_id=r), 1)

        # TODO: adjust rate
        self.timer = self.create_timer(0.2, self.step_flocking)

    def battery_callback(self, msg: BatteryState, robot_id: int):
        self.voltage[robot_id] = msg.voltage

    def step_flocking(self):
        for r in self.robots:
            goal = Point()
            goal.x = 1.0
            goal.y = 1.0
            self.goal_pubs[r].publish(goal)