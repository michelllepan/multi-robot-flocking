import numpy as np
import rospy
from geometry_msgs.msg import Twist
from scipy.spatial.transform import Rotation

from flocking.ros.publishers import VelocityPublisher
from flocking.ros.subscribers import JointStateSubscriber, OdomSubscriber


class Robot:

    def __init__(self, robot_id):
        self.joint_state_sub = JointStateSubscriber(robot_id)
        self.odom_sub = OdomSubscriber(robot_id)
        self.cmd_vel_pub = VelocityPublisher(robot_id)

        self.rate = rospy.Rate(10.0)
        self.turn_direction = "left"
        self.goal_reached = True

        print("initializing odom")
        while not self.update_odom():
            self.update_odom()

    def update_odom(self):
        if self.odom_sub.data is None: return False

        self.x = self.odom_sub.position.x
        self.y = self.odom_sub.position.y

        quat = Rotation.from_quat([
            self.odom_sub.orientation.x,
            self.odom_sub.orientation.y,
            self.odom_sub.orientation.z,
            self.odom_sub.orientation.w])
        euler = quat.as_euler("xyz") # radians
        self.h = euler[2]

        return True

    def set_goal(self, x=None, y=None, heading=None):
        self.goal_x = x
        self.goal_y = y
        self.goal_h = heading
        # self.choose_direction()
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
        twist.linear.x = 0.7
        # print(self.x, self.y)
        if cross > 0.01:
            print("going right")
            twist.angular.z = -2 * (theta / np.pi)
        elif cross < -0.01:
            print("going left")
            twist.angular.z = 2 * (theta / np.pi)
        else:
            print("going straight")
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

    # def turn_to(self, heading):
    #     while not self.odom_initialized:
    #         self.update_odom()

    #     angle_to_right = (self.h - (heading - 2 * np.pi)) % (2 * np.pi)
    #     angle_to_left = ((heading + 2 * np.pi) - self.h) % (2 * np.pi)

    #     twist = Twist()
    #     twist.linear.x = 0.0
    #     if angle_to_right < angle_to_left:
    #         print("turning right")
    #         twist.angular.z = -0.5
    #     else:
    #         print("turning left")
    #         twist.angular.z = 0.5

    #     while not self.check_at_heading(heading):
    #         self.cmd_vel_pub.publish(twist)
    #         self.rate.sleep()
    #         self.update_odom()