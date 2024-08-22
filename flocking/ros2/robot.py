import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import BatteryState, LaserScan


class Robot(Node):

    def __init__(self, robot_id):
        robot_name = "robot_" + str(robot_id)
        super().__init__(robot_name)

        # publishers
        self.vel_pub = self.create_publisher(
            Twist, f"/{robot_name}/stretch/cmd_vel", 1)

        # subscribers
        self.goal_sub = self.create_subscription(
            Point, f"/{robot_name}/goal", self.goal_callback, 1)
        self.scan_sub = self.create_subscription(
            LaserScan, f"/{robot_name}/scan", self.scan_callback, 1)

    def goal_callback(self, msg):
        print(msg)

    def scan_callback(self, msg):
        pass