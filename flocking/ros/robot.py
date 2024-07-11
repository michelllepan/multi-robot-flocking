import rospy
from geometry_msgs.msg import Twist

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
        
    def set_goal(self, x, y, heading=None):
        self.goal_x = x
        self.goal_y = y
        self.goal_h = heading

    def check_at_goal(self, tolerance=1e-2) -> bool:
        if self.odom_sub.data is None: return False
        return (
            abs(self.goal_x - self.odom_sub.position.x) < tolerance and
            abs(self.goal_y - self.odom_sub.position.y) < tolerance and
            abs(self.goal_h - self.odom_sub.orientation.z) < tolerance 
                if self.goal_h is not None else True
        )
    
    def move_towards_goal(self):
        x = self.odom_sub.position.x
        y = self.odom_sub.position.y

        # ignore heading for now
        # h = self.odom_sub.orientation.z

        

