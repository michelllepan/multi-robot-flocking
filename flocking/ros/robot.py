import math

import numpy as np
import rospy
from geometry_msgs.msg import Point, Twist
from scipy.spatial.transform import Rotation

from flocking.ros.publishers import TargetPublisher, VelocityPublisher
from flocking.ros.subscribers import GroundTruthSubscriber, JointStateSubscriber


MAX_LIN_ACCEL = 0.7
MAX_ANG_ACCEL = 0.1

class Robot:

    def __init__(self, robot_id):
        self.robot_id = robot_id

        self.joint_state_sub = JointStateSubscriber(robot_id)
        self.gt_sub = GroundTruthSubscriber(robot_id)
        self.target_pub = TargetPublisher(robot_id)
        self.cmd_vel_pub = VelocityPublisher(robot_id)

        self.turn_direction = "left"
        self.lin_vel = 0
        self.ang_vel = 0
        self.goal_reached = True

        print("initializing odom")
        while not self.update_odom():
            self.update_odom()

    def update_odom(self):
        if self.gt_sub.data is None: return False

        self.x = self.gt_sub.position.x
        self.y = self.gt_sub.position.y

        quat = Rotation.from_quat([
            self.gt_sub.orientation.x,
            self.gt_sub.orientation.y,
            self.gt_sub.orientation.z,
            self.gt_sub.orientation.w])
        euler = quat.as_euler("xyz") # radians
        self.h = euler[2]

        return True

    def set_goal(self, x=None, y=None, heading=None):
        self.goal_x = x
        self.goal_y = y
        self.goal_h = heading

        point = Point()
        point.x = x
        point.y = y
        self.target_pub.publish(point)

        self.goal_reached = False

    def choose_direction(self):
        angle_to_right = (self.h - (self.goal_h - 2 * np.pi)) % (2 * np.pi)
        angle_to_left = ((self.goal_h + 2 * np.pi) - self.h) % (2 * np.pi)

        if angle_to_right < angle_to_left:
            self.turn_direction = "right"
        else:
            self.turn_direction = "left"

    def check_at_pos_goal(self, tolerance=0.1) -> bool:
        if self.goal_x is None or self.goal_y is None: return False
        self.goal_reached = (self.goal_reached or 
                             (abs(self.goal_x - self.x) < tolerance and
                              abs(self.goal_y - self.y) < tolerance))
        return self.goal_reached
    
    def check_at_heading_goal(self, tolerance=0.1) -> bool:
        if self.goal_h is None: return False
        self.goal_reached = (self.goal_reached or
                             abs(self.h - self.goal_h) < tolerance)
        return self.goal_reached
    
    def move_toward_goal(self):
        # TODO: align to goal heading
        heading_vec = np.array([np.cos(self.h), np.sin(self.h)])
        goal_vec = np.array([self.goal_x - self.x, self.goal_y - self.y])

        # cross > 0: goal is to the right of robot
        # cross < 0: goal is to the left of robot
        cross = np.cross(goal_vec, heading_vec)

        # find angle between heading and direction of goal
        theta = np.arccos(np.dot(heading_vec, goal_vec) / 
                          (np.linalg.norm(heading_vec) * np.linalg.norm(goal_vec)))

        twist = Twist()

        # set linear velocity
        if theta < np.pi / 2:
            new_lin_vel = min(10 * np.linalg.norm(goal_vec), 7.0) 
        else:
            new_lin_vel = 0

        lin_accel = new_lin_vel - self.lin_vel
        lin_accel = math.copysign(min(abs(lin_accel), MAX_LIN_ACCEL), lin_accel)
        self.lin_vel += lin_accel
        twist.linear.x = self.lin_vel

        # set angular velocity
        if cross > 0.01:
            new_ang_vel = -min(2 * (theta / np.pi), 1.0)
        elif cross < -0.01:
            new_ang_vel = min(2 * (theta / np.pi), 1.0)
        else:
            new_ang_vel = 0

        ang_accel = new_ang_vel - self.ang_vel
        ang_accel = math.copysign(min(abs(ang_accel), MAX_ANG_ACCEL), ang_accel)
        self.ang_vel += ang_accel
        twist.angular.z = self.ang_vel

        print(f"ROBOT {self.robot_id}   goal: [{goal_vec[0] : 5.2f} {goal_vec[1] : 5.2f}] | heading: {self.h : 5.2f} | twist_x: {twist.linear.x : 5.2f} | twist_z: {twist.angular.z : 5.2f}")

        self.cmd_vel_pub.publish(twist)

    def turn_toward_goal(self):
        twist = Twist()
        twist.linear.x = 0.0

        if self.turn_direction == "right":
            twist.angular.z = -0.5
        else:
            twist.angular.z = 0.5

        self.cmd_vel_pub.publish(twist)

    def print_odometry(self):
        print(f"ROBOT {self.robot_id}    x: {self.x : 5.2f}    y: {self.y : 5.2f}    heading: {self.h : 5.2f}")
