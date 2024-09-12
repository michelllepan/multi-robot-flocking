import os
from io import BytesIO

import numpy as np
import redis
from PIL import Image
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, JointState, LaserScan
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from flocking.humans import HumanTracker
from flocking.utils import Pose


REDIS_HOST = "10.5.90.8"
REDIS_PORT = "6379"

class StatePublisher(Node):

    def __init__(self, robot_id):
        super().__init__("state_publisher")

        robot_name = "robot_" + str(robot_id)
        self.get_logger().info("creating state publisher")

        # redis
        self.redis_client = redis.Redis(host=REDIS_HOST, port=REDIS_PORT)
        self.pose_key = robot_name + "::pose"
        self.battery_key = robot_name + "::battery"
        self.obstacles_front_key = robot_name + "::obstacles::front"
        self.obstacles_back_key = robot_name + "::obstacles::back"
        self.humans_key = robot_name + "::humans"
        self.image_key = robot_name + "::image" 
        self.head_key = robot_name + "::head"

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
        def human_callback(humans, image):
            self.publish_humans(humans)
            self.publish_image(image)

        # head
        self.joint_states_sub = self.create_subscription(
            JointState, '/joint_states', self.publish_head, 1)
        self.head = None

        self.human_tracker = HumanTracker(human_callback=human_callback)
        self.human_timer = self.create_timer(0.33, self.human_tracker.process_frame)

        self.get_logger().info("created state publisher")

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
        points = [r * np.sin(theta) if (theta < -2.5 or theta > 2.5) else np.inf for r,theta in zip(msg.ranges, angles)]

        # If we're close to the x axis, keep the range, otherwise use inf, which means "no return"
        new_ranges = [r if abs(y) < 0.5 else np.inf for r,y in zip(msg.ranges, points)]

        # If closest measured scan is within obstacle threshold, stop
        obstacle_present = min(new_ranges) < 0.5
        self.redis_client.set(self.obstacles_front_key, str(obstacle_present))

        # do the same but for the back
        points = [r * np.sin(theta) if (theta < 0.5 and theta > -0.5) else np.inf for r,theta in zip(msg.ranges, angles)]
        new_ranges = [r if abs(y) < 0.5 else np.inf for r,y in zip(msg.ranges, points)]
        obstacle_present = min(new_ranges) < 0.5
        self.redis_client.set(self.obstacles_back_key, str(obstacle_present))

    def publish_humans(self, detections_robot):
        if self.pose is None or self.head is None or detections_robot is None:
            return
        elif len(detections_robot) == 0:
            self.redis_client.set(self.humans_key, str(detections_robot))
            return

        detections_robot = np.array(detections_robot)
        if detections_robot.ndim < 2:
            detections_robot = np.expand_dims(detections_robot, axis=0)

        # add z coordinate for rotation
        z = np.zeros((detections_robot.shape[0], 1))
        detections_robot = np.hstack((detections_robot, z))

        # rotation
        rot = Rotation.from_euler("z", self.pose.h + self.head)
        detections_world = rot.apply(detections_robot)
        detections_world = detections_world[:, :2]

        # translation
        detections_world += np.array([self.pose.x, self.pose.y])

        detections_world = detections_world.tolist()
        self.redis_client.set(self.humans_key, str(detections_world))

    def publish_image(self, image):
        output = BytesIO()
        image = image[:,:,::-1] # switch order of color channels
        image = Image.fromarray(image.astype("uint8"), "RGB")
        image.save(output, format="png")

        self.redis_client.set(self.image_key, output.getvalue())
        output.close()

    def publish_head(self, msg: JointState):
        index = msg.name.index("joint_head_pan")
        value = msg.position[index]
        self.head = value
        self.redis_client.set(self.head_key, self.head)
        
