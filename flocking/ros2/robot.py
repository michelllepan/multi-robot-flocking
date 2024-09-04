import numpy as np
import redis
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, LaserScan

from flocking.utils import Goal, Pose


GOAL_TOLERANCE = 0.1
OBS_TOLERANCE = 0.5

LIN_VEL_SCALE = 0.5
ANG_VEL_SCALE = 1.0

# MAX_ANG_SPEED = 1.0 

REDIS_HOST = "10.5.90.8"
REDIS_PORT = "6379"

class Robot(Node):

    def __init__(self, robot_id):
        robot_name = "robot_" + str(robot_id)
        super().__init__(robot_name)

        # publishers
        self.vel_pub = self.create_publisher(
            Twist, "/stretch/cmd_vel", 1)

        # subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 1)

        # redis
        self.redis_client = redis.Redis(host=REDIS_HOST, port=REDIS_PORT)
        self.goal_timer = self.create_timer(0.1, self.redis_get_goal)

        self.goal_key = robot_name + "::goal"
        self.pose_key = robot_name + "::pose"

        # initialize state
        self.pose = None
        self.goal = None
        self.twist = Twist()

        # tf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_timer = self.create_timer(0.001, self.extract_pose)
        self.extract_pose()

        # clear old goal
        self.redis_update_pose()
        self.redis_client.set(self.goal_key, str(Goal(x=0.0, y=0.0)))
  
        self.obstacle_present = False

    ### POSE METHODS

    def print_pose(self):
        print(f"x: {self.pose.x : 5.2f}    y: {self.pose.y : 5.2f}    heading: {self.pose.h : 5.2f}     lin vel: {self.twist.linear.x : 5.2f}    ang vel: {self.twist.angular.z : 5.2f}")

    def extract_pose(self):
        to_frame = "map"
        from_frame = "base_link"

        try:
            t = self.tf_buffer.lookup_transform(
                    to_frame,
                    from_frame,
                    rclpy.time.Time())
            # print(t)
        except TransformException as ex:
            self.get_logger().info(
                f"Could not transform {to_frame} to {from_frame}: {ex}")
            return

        position = t.transform.translation
        orientation = t.transform.rotation

        quat = Rotation.from_quat([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w])
        euler = quat.as_euler("xyz") # radians

        self.pose = Pose(
            x=position.x,
            y=position.y,
            h=float(euler[2]))

        self.redis_update_pose()
        self.move_toward_goal()
        self.print_pose()

    ### GOAL METHODS

    def check_at_goal(self) -> bool:
        if self.pose is None or self.goal is None:
            return False
        return (abs(self.pose.x - self.goal.x) < GOAL_TOLERANCE and 
                abs(self.pose.y - self.goal.y) < GOAL_TOLERANCE)

    def move_toward_goal(self):
        if self.pose is None or self.goal is None: return
        if self.check_at_goal(): return
        if self.obstacle_present:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.vel_pub.publish(self.twist)
            return 

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
        if theta < np.pi / 2:
            self.twist.linear.x = min(np.tanh(LIN_VEL_SCALE * np.linalg.norm(goal_vec)), 0.3)
        else:
            self.twist.linear.x = 0.0

        # calculate angular velocity
        angular_speed = ANG_VEL_SCALE * theta
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
        # print("publishing")
        # print(self.twist)

    ### SUBSCRIBER CALLBACKS

    def scan_callback(self, msg: LaserScan):
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))

        # Work out the y coordinates of the ranges
        points = [r * np.sin(theta) if (theta < -2.5 or theta > 2.5) else np.inf for r,theta in zip(msg.ranges, angles)]

        # If we're close to the x axis, keep the range, otherwise use inf, which means "no return"
        new_ranges = [r if abs(y) < 0.5 else np.inf for r,y in zip(msg.ranges, points)]

        # If closest measured scan is within obstacle threshold, stop
        self.obstacle_present = min(new_ranges) < OBS_TOLERANCE

        if self.obstacle_present:
            print("obstacle detected")

    ### REDIS LISTENER

    def redis_get_goal(self):
        goal_str = self.redis_client.get(self.goal_key)
        if goal_str:
            self.goal = Goal.from_string(goal_str)

    def redis_update_pose(self):
        self.redis_client.set(self.pose_key, str(self.pose))
