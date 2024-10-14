import numpy as np
import redis
import time
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


GOAL_TOLERANCE = 0.05
OBS_TOLERANCE = 0.75

LIN_VEL_SCALE = 1.5
LIN_VEL_MAX = 0.3

ANG_VEL_SCALE = 0.6
ANG_VEL_MAX = 1.0

REDIS_PORT = "6379"

class FlockFollower(Node):

    def __init__(self, robot_id, redis_host):
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
        self.redis_client = redis.Redis(host=redis_host, port=REDIS_PORT)
        self.redis_timer = self.create_timer(0.005, self.read_redis)

        self.flock_state_key = "state"
        self.goal_key = robot_name + "::goal"
        self.pose_key = robot_name + "::pose"
        self.obstacles_front_key = robot_name + "::obstacles::front"
        self.obstacles_back_key = robot_name + "::obstacles::back"
        self.obstacles_side_key = robot_name + "::obstacles::side"
        self.look_key = robot_name + "::look"
        self.arm_key = robot_name + "::arm"

        # initialize state
        self.flock_state = "STOP"
        self.pose = None
        self.goal = None
        self.obstacle_front = False
        self.obstacle_back = False
        self.obstacle_side = False
        self.look = None
        self.arm = None

        # base movement
        self.move_base_timer = self.create_timer(0.005, self.move_base)
        self.twist = Twist()

        # joint movement
        self.move_joints_timer = self.create_timer(0.05, self.move_joints)
        self.trajectory_client = ActionClient(self,
            FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory')
        server_reached = self.trajectory_client.wait_for_server(timeout_sec=10.0)
        if not server_reached:
            self.get_logger().error('Unable to connect to server. Timeout exceeded. Is stretch_driver running?')
            sys.exit()
        
    def read_redis(self):
        try:
            flock_state_str = self.redis_client.get(self.flock_state_key)
            self.flock_state = flock_state_str.decode("utf-8") if flock_state_str else "STOP"

            pose_str = self.redis_client.get(self.pose_key)
            self.pose = Pose.from_string(pose_str)

            goal_str = self.redis_client.get(self.goal_key)
            self.goal = Goal.from_string(goal_str)

            obstacle_str = self.redis_client.get(self.obstacles_front_key)
            self.obstacle_front = eval(obstacle_str) if obstacle_str and obstacle_str != b"inf" else float("inf")

            obstacle_str = self.redis_client.get(self.obstacles_back_key)
            self.obstacle_back = eval(obstacle_str) if obstacle_str and obstacle_str != b"inf" else float("inf")

            obstacle_str = self.redis_client.get(self.obstacles_side_key)
            self.obstacle_side = eval(obstacle_str) if obstacle_str and obstacle_str != b"inf" else float("inf")

            look_str = self.redis_client.get(self.look_key)
            self.look = eval(look_str) if look_str else None

            arm_str = self.redis_client.get(self.arm_key)
            self.arm = eval(arm_str) if arm_str else None
        except redis.exceptions.ConnectionError as e:
            print(e, f" at time {time.time() : .0f}")

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
        if self.pose is None or self.goal is None:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.vel_pub.publish(self.twist)
            return
        
        if self.flock_state == "STOP":
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.vel_pub.publish(self.twist)
            return
        elif self.flock_state == "GESTURE_SPIN":
            self.twist.linear.x = 0.0
            self.twist.angular.z = 1.0
            self.vel_pub.publish(self.twist)
            return
        
        max_lin_speed = 0.3
        if self.obstacle_front < 0.75:
            if self.obstacle_back < 0.75:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.vel_pub.publish(self.twist)
                print("ROBOT BLOCKED: STOPPING !!!!1!!11!111!!!!!!!!!")
                return
            else:
                self.twist.linear.x = -0.1 
                self.twist.angular.z = 0.0
                self.vel_pub.publish(self.twist)
                print("BACKING UP")
                return 
        elif self.obstacle_front < 1.5:
            max_lin_speed = 0.1

        if self.check_at_goal():
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.vel_pub.publish(self.twist)
            return
            # if self.goal.h is not None and abs(self.pose.h - self.goal.h) > 0.05:
            #     self.twist.linear.x = 0.0
            #     self.twist.angular.z = 0.1
            #     self.vel_pub.publish(self.twist)
            # else:
            #     self.twist.linear.x = 0.0
            #     self.twist.angular.z = 0.0
            #     self.vel_pub.publish(self.twist)
            #     return

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
        if self.flock_state == "GOTO":
            linear_speed = 0.15 * np.tanh(4 * np.linalg.norm(goal_vec) - 2.0) + 0.15
        else:
            linear_speed = 0.15 * np.tanh(30 * np.linalg.norm(goal_vec) - 4.5) + 0.15 
        if theta < np.pi / 2:
            self.twist.linear.x = min(linear_speed, max_lin_speed)
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

        # publish twist
        self.vel_pub.publish(self.twist)

    def move_joints(self):
        joint_names1, pos1, vel1 = self.move_head()
        joint_names2, pos2, vel2 = self.move_arm()

        point = JointTrajectoryPoint()
        point.time_from_start = Duration(seconds=1.0).to_msg()
        point.positions = pos1 + pos2
        point.velocities = vel1 + vel2

        trajectory_goal = FollowJointTrajectory.Goal()
        trajectory_goal.trajectory.joint_names = joint_names1 + joint_names2
        trajectory_goal.trajectory.points = [point]
        self.trajectory_client.send_goal_async(trajectory_goal)

    def move_head(self):
        if self.look is None or self.head is None:
            return [], [], []

        look = self.look - self.pose.h
        look = (look + np.pi) % (2 * np.pi) - np.pi  # wrap to between -pi and pi

        if abs(look - self.head) < 0.05:
            return [], [], []

        if look - self.head < 0:
            target = max(-0.3, look - self.head)
        else:
            target = min(0.3, look - self.head)

        return ["joint_head_pan"], [self.head + target], [0.15 * np.tanh(30 * abs(target) - 3.5) + 0.15]

    def move_arm(self):
        if self.arm is None:
            return [], [], []

        if self.obstacle_side < 0.75:
            retract = {
                "arm_l3": (0.0, 0.2),
                "arm_l2": (0.0, 0.2),
                "arm_l1": (0.0, 0.2),
                "arm_l0": (0.0, 0.2),
                "wrist_yaw": (3.0, 0.4),
                "wrist_pitch": (-1.0, 0.4),
            }
            self.arm.update(retract)
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
        
