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

        self.goal_x = None
        self.goal_y = None
        self.goal_h = None

    def update_odom(self):
        if self.odom_sub.position is not None:
            self.x = self.odom_sub.position.x
            self.y = self.odom_sub.position.y

        if self.odom_sub.orientation is not None:
            quat = Rotation.from_quat([
                self.odom_sub.orientation.x,
                self.odom_sub.orientation.y,
                self.odom_sub.orientation.z,
                self.odom_sub.orientation.w])
            euler = quat.as_euler("xyz") # radians
            self.h = euler[2]

    def set_goal(self, x, y, heading=None):
        self.goal_x = x
        self.goal_y = y
        self.goal_h = heading

    def check_at_goal(self, tolerance=0.1) -> bool:
        if self.odom_sub.data is None: return False
        return (
            abs(self.goal_x - self.x) < tolerance and
            abs(self.goal_y - self.y) < tolerance and
            (abs(self.goal_h - self.h) < tolerance 
                if self.goal_h is not None else True)
        )
    
    def move_towards_goal(self):
        if self.odom_sub.data is None: return

        # TODO: align to goal heading
        # TODO: handle goals behind the robot
        direction_vec = np.array([np.cos(self.h), np.sin(self.h)])
        goal_vec = np.array([self.goal_x - self.x, self.goal_y - self.y])
        # cross > 0: goal is to the right of robot
        # cross < 0: goal is to the left of robot
        cross = np.cross(goal_vec, direction_vec)

        twist = Twist()
        twist.linear.x = 0.2
        print(x, y)
        if cross > 0.01:
            print("turning right")
            twist.angular.z = -0.2
        elif cross < -0.01:
            print("turning left")
            twist.angular.z = 0.2
        else:
            print("going straight")
            twist.angular.z = 0

        self.cmd_vel_pub.publish(twist)

        

        

