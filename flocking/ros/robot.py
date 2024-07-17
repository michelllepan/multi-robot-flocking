import numpy as np
import rospy
from geometry_msgs.msg import Point, Twist
from scipy.spatial.transform import Rotation

from flocking.ros.publishers import TargetPublisher, VelocityPublisher
from flocking.ros.subscribers import GroundTruthSubscriber, JointStateSubscriber


class Robot:

    def __init__(self, robot_id):
        self.joint_state_sub = JointStateSubscriber(robot_id)
        self.gt_sub = GroundTruthSubscriber(robot_id)
        self.target_pub = TargetPublisher(robot_id)
        self.cmd_vel_pub = VelocityPublisher(robot_id)

        self.rate = rospy.Rate(10.0)
        self.turn_direction = "left"
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
        
        if np.cos(theta) != 0: 
            twist.linear.x = 2.0 * goal_vec[0] / np.cos(theta)
        else:
            twist.linear.x = 2.0 * goal_vec[1] / np.sin(theta)

        if cross > 0.01:
            twist.angular.z = -1.2 * (theta / np.pi)
        elif cross < -0.01:
            twist.angular.z = 1.2 * (theta / np.pi)
        else:
            twist.angular.z = 0

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
        print(f"x: {self.x : 5.2f}    y: {self.y : 5.2f}    heading: {self.h : 5.2f}")
