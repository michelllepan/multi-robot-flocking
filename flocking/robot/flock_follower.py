import numpy as np
import redis
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, JointState, LaserScan
from trajectory_msgs.msg import JointTrajectoryPoint

from flocking.utils import Goal, Pose


GOAL_TOLERANCE = 0.1
OBS_TOLERANCE = 0.5

LIN_VEL_SCALE = 0.5
LIN_VEL_MAX = 0.3

ANG_VEL_SCALE = 0.6
ANG_VEL_MAX = 1.0

REDIS_HOST = "10.5.90.8"
REDIS_PORT = "6379"


class FlockFollower(Node):

    def __init__(self, robot_id):
        robot_name = "robot_" + str(robot_id)
        super().__init__(robot_name)

        # publishers
        self.vel_pub = self.create_publisher(
            Twist, "/stretch/cmd_vel", 1)

        # head
        self.joint_states_sub = self.create_subscription(
            JointState, '/stretch/joint_states', self.read_head, 1)
        self.head = None

        # redis
        self.redis_client = redis.Redis(host=REDIS_HOST, port=REDIS_PORT)
        self.redis_timer = self.create_timer(0.01, self.read_redis)

        self.goal_key = robot_name + "::goal"
        self.pose_key = robot_name + "::pose"
        self.obstacles_front_key = robot_name + "::obstacles::front"
        self.obstacles_back_key = robot_name + "::obstacles::back"
        self.look_key = robot_name + "::look"
        self.arm_key = robot_name + "::arm"

        # initialize state
        self.pose = None
        self.goal = None
        self.obstacle_present_front = False
        self.obstacle_present_back = False
        self.look = None
        self.arm = None

        # base movement
        self.move_base_timer = self.create_timer(0.01, self.move_base)
        self.twist = Twist()

        # joint movement
        self.move_joints_timer = self.create_timer(0.1, self.move_joints)
        self.trajectory_client = ActionClient(self,
            FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory')
        server_reached = self.trajectory_client.wait_for_server(timeout_sec=10.0)
        if not server_reached:
            self.get_logger().error('Unable to connect to server. Timeout exceeded. Is stretch_driver running?')
            sys.exit()
        
    def read_redis(self):
        pose_str = self.redis_client.get(self.pose_key)
        self.pose = Pose.from_string(pose_str)

        goal_str = self.redis_client.get(self.goal_key)
        self.goal = Goal.from_string(goal_str)

        obstacle_str = self.redis_client.get(self.obstacles_front_key)
        self.obstacle_present_front = eval(obstacle_str)

        look_str = self.redis_client.get(self.look_key)
        self.look = eval(look_str) if look_str else None

        arm_str = self.redis_client.get(self.arm_key)
        self.arm = eval(arm_str) if arm_str else None

        self.print_info()

    def read_head(self, msg: JointState):
        index = msg.name.index("joint_head_pan")
        value = msg.position[index]
        self.head = value

    def print_info(self):
        print(f"x: {self.pose.x : 5.2f}    y: {self.pose.y : 5.2f}    heading: {self.pose.h : 5.2f}     lin vel: {self.twist.linear.x : 5.2f}    ang vel: {self.twist.angular.z : 5.2f}")

    def check_at_goal(self) -> bool:
        if self.pose is None or self.goal is None:
            return False
        return (abs(self.pose.x - self.goal.x) < GOAL_TOLERANCE and 
                abs(self.pose.y - self.goal.y) < GOAL_TOLERANCE)

    def move_base(self):
        # return
        if self.pose is None or self.goal is None: return
        if self.obstacle_present_front:
            if self.obstacle_present_back:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.vel_pub.publish(self.twist)
                print("ROBOT BLOCKED: STOPPING !!!!1!!11!111!!!!!!!!!")
            else:
                self.twist.linear.x = -0.2
                self.twist.angular.z = 0.0
                self.vel_pub.publish(self.twist)
                print("BACKING UP")
            return 
        if self.check_at_goal(): return

        # unit vector of the heading
        heading_vec = np.array([np.cos(self.pose.h), np.sin(self.pose.h)])
        # vector from the robot to goal
        goal_vec = np.array([self.goal.x - self.pose.x, self.goal.y - self.pose.y])

        # cross > 0: goal is to the right of robot
        # cross < 0: goal is to the left of robot
        cross = np.cross(goal_vec, heading_vec)

        # find angle between heading and direction of goal
        theta = np.arccos(np.dot(heading_vec, goal_vec) / 
                          (np.linalg.norm(heading_vec) * np.linalg.norm(goal_vec)))

        # calculate linear velocity
        linear_speed = np.tanh(LIN_VEL_SCALE * np.linalg.norm(goal_vec))
        if theta < np.pi / 2:
            self.twist.linear.x = min(linear_speed, 0.3)
        else:
            self.twist.linear.x = 0.0

        # calculate angular velocity
        angular_speed = np.tanh(ANG_VEL_SCALE * theta)
        if self.twist.linear.x == 0.0:
            angular_speed = min(angular_speed, 0.5)
        else:
            angular_speed = min(angular_speed, 1.0)
        if cross > 0.01:
            self.twist.angular.z = -angular_speed
        elif cross < -0.01:
            self.twist.angular.z = angular_speed
        else:
            self.twist.angular.z = 0.0

        # publish twist
        self.vel_pub.publish(self.twist)

    def move_joints(self):
        # head_pos, head_vel = self.move_head()
        joint_names, pos, vel = self.move_arm()

        point = JointTrajectoryPoint()
        point.time_from_start = Duration(seconds=1.0).to_msg()
        point.positions = pos
        point.velocities = vel

        trajectory_goal = FollowJointTrajectory.Goal()
        trajectory_goal.trajectory.joint_names = joint_names
        trajectory_goal.trajectory.points = [point]
        self.trajectory_client.send_goal_async(trajectory_goal)

    def move_head(self):
        if self.look is None or self.head is None:
            return {}

        look = self.look - self.pose.h
        look = (look + np.pi) % (2 * np.pi) - np.pi  # wrap to between -pi and pi

        if abs(look - self.head) < 0.1:
            return {}

        # point = JointTrajectoryPoint()
        # point.time_from_start = Duration(seconds=10.0).to_msg()

        if look - self.head < 0:
            target = max(-0.2, look - self.head)
        else:
            target = min(0.2, look - self.head)
        # point.positions = [self.head + target]

        return {"joint_head_pan": self.head + target}

    def move_arm(self):
        if self.arm is None:
            return {}
        
        """
        - joint_lift
        - joint_arm_l3
        - joint_arm_l2
        - joint_arm_l1
        - joint_arm_l0
        - joint_wrist_yaw
        - joint_wrist_pitch
        - joint_wrist_roll
        - joint_gripper_finger_left
        - joint_gripper_finger_right
        """
        pos = [value[0] for value in self.arm.values()]
        vel = [value[1] for value in self.arm.values()]
        joint_names = ["joint_" + key for key in self.arm.keys()]
        return joint_names, pos, vel
        
