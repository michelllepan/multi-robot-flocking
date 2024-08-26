from dataclasses import dataclass

from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, LaserScan

from .utils import Goal, Odom


GOAL_TOLERANCE = 0.1
LIN_VEL_SCALE = 1
ANG_VEL_SCALE = 1

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
        self.odom_sub = self.create_subscription(
            Odometry, f"/{robot_name}/odom", self.odom_callback, 1)
        self.scan_sub = self.create_subscription(
            LaserScan, f"/{robot_name}/scan", self.scan_callback, 1)

        # initialize state
        self.odom = None
        self.goal = None
        self.twist = Twist()

    def check_at_goal(self) -> bool:
        if self.odom is None or self.goal is None:
            return False
        return (abs(self.odom.x - self.goal.x) < GOAL_TOLERANCE and 
                abs(self.odom.y - self.goal.y) < GOAL_TOLERANCE)

    def move_toward_goal(self):
        if self.odom is None or self.goal is None: return
        if self.check_at_goal(): return

        # unit vector of the heading
        heading_vec = np.array([np.cos(self.odom.h), np.sin(self.odom.h)])
        # vector from the robot to goal
        goal_vec = np.array([self.goal.x - self.odom.x, self.goal.y - self.odom.y])

        # cross > 0: goal is to the right of robot
        # cross < 0: goal is to the left of robot
        cross = np.cross(goal_vec, heading_vec)

        # find angle between heading and direction of goal
        theta = np.arccos(np.dot(heading_vec, goal_vec) / 
                          (np.linalg.norm(heading_vec) * np.linalg.norm(goal_vec)))

        # calculate linear velocity
        if theta < np.pi / 2:
            self.twist.linear.x = np.tanh(LIN_VEL_SCALE * np.linalg.norm(goal_vec))
        else:
            self.twist.linear.x = 0

        # calculate angular velocity
        if cross > 0.01:
            self.twist.angular.z = -ANG_VEL_SCALE * theta
        elif cross < -0.01:
            self.twist.angular.z = ANG_VEL_SCALE * theta
        else:
            self.twist.angular.z = 0

        # publish twist
        self.vel_pub.publish(self.twist)

    def print_odometry(self):
        print(f"x: {self.odom.x : 5.2f}    y: {self.odom.y : 5.2f}    heading: {self.odom.h : 5.2f}")

    ### SUBSCRIBER CALLBACKS

    def goal_callback(self, msg: Point):
        self.goal = Goal(
            x=msg.x,
            y=msg.y)

    def odom_callback(self, msg: Odometry):
        position = data.pose.pose.position
        orientation = data.pose.pose.orientation

        quat = Rotation.from_quat([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w])
        euler = quat.as_euler("xyz") # radians

        self.odom = Odom(
            x=position.x,
            y=position.y,
            h=euler[2])
        
        self.move_toward_goal()

    def scan_callback(self, msg: LaserScan):
        pass