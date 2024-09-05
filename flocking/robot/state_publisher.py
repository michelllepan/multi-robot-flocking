import os

import numpy as np
import redis
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from sensor_msgs.msg import BatteryState, LaserScan

from flocking.humans import HumanTracker
from flocking.utils import Pose


REDIS_HOST = "10.5.90.8"
REDIS_PORT = "6379"

class StatePublisher(Node):

    def __init__(self, robot_id):
        super().__init__("state_publisher")

        robot_name = "robot_" + str(robot_id)
        print("creating state publisher")

        # redis
        self.redis_client = redis.Redis(host=REDIS_HOST, port=REDIS_PORT)
        self.pose_key = robot_name + "::pose"
        self.battery_key = robot_name + "::battery"
        self.obstacles_key = robot_name + "::obstacles"
        self.humans_key = robot_name + "::humans"

        # pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_timer = self.create_timer(0.001, self.publish_pose)
        self.pose = None

        # battery
        self.battery_sub = self.create_subscription(
            BatteryState, "/battery", self.publish_battery, 1)

        # scan
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.publish_obstacles, 1)

        # humans
        self.human_tracker = HumanTracker()
        self.human_timer = self.create_timer(0.1, self.publish_humans)

        print("created state publisher")

    def publish_pose(self):
        to_frame, from_frame = "map", "base_link"
        try:
            t = self.tf_buffer.lookup_transform(
                    to_frame,
                    from_frame,
                    rclpy.time.Time())
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
        self.redis_client.set(self.pose_key, str(self.pose))

    def publish_battery(self, msg: BatteryState):
        voltage = msg.voltage
        self.redis_client.set(self.battery_key, voltage)

    def publish_obstacles(self, msg: LaserScan):
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))

        # Work out the y coordinates of the ranges
        points = [r * np.sin(theta) if (theta < -2.0 or theta > 2.0) else np.inf for r,theta in zip(msg.ranges, angles)]

        # If we're close to the x axis, keep the range, otherwise use inf, which means "no return"
        new_ranges = [r if abs(y) < 0.75 else np.inf for r,y in zip(msg.ranges, points)]

        # If closest measured scan is within obstacle threshold, stop
        obstacle_present = min(new_ranges) < 0.75
        self.redis_client.set(self.obstacles_key, str(obstacle_present))

    def publish_humans(self):
        if self.pose is None:
            print("no pose")
            return

        print("processing frame")
        detections_robot = self.human_tracker.process_frame()
        if not detections_robot:
            print("no detections")
            return

        detections_robot = np.array(detections_robot)
        if detections_robot.ndim < 2:
            detections_robot = np.expand_dims(detections_robot, axis=0)

        # add z coordinate for rotation
        z = np.zeros((detections_robot.shape[0], 1))
        detections_robot = np.hstack((detections_robot, z))

        # rotation
        rot = Rotation.from_euler("z", self.pose.h)
        detections_world = rot.apply(detections_robot)
        detections_world = detections_world[:, :2]

        # translation
        detections_world += np.array([self.pose.x, self.pose.y])

        detections_world = detections_world.tolist()
        print(detections_world)
        self.redis_client.set(self.humans_key, str(detections_world))
