import os
from io import BytesIO

import numpy as np
import redis
from PIL import Image
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState, JointState, LaserScan
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from flocking.humans import HumanTracker
from flocking.utils import Pose


REDIS_HOST = "192.168.1.150"
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
        self.obstacles_side_key = robot_name + "::obstacles::side"
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
        self.human_timer = self.create_timer(0.25, self.human_tracker.process_frame)

        # music
        self.music_base_sub = self.create_subscription(
            Twist, '/stretch/cmd_vel', self.publish_music_base, 1)
        self.music_joints_sub = self.create_subscription(
            JointState, '/joint_states', self.publish_music_joints, 1)

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

        ### FRONT
        # work out the y coordinates of the ranges
        points = [r * np.sin(theta) if (theta < -np.pi/2 or theta > np.pi/2) else np.inf for r,theta in zip(msg.ranges, angles)]
        # if we're close to the x axis, keep the range, otherwise use inf, which means "no return"
        new_ranges = [r if abs(y) < 0.5 else np.inf for r,y in zip(msg.ranges, points)]
        # publish distance to closest point
        self.redis_client.set(self.obstacles_front_key, str(min(new_ranges)))

        ### BACK
        points = [r * np.sin(theta) if (theta < 0.9 and theta > -0.9) else np.inf for r,theta in zip(msg.ranges, angles)]
        new_ranges = [r if abs(y) < 0.5 else np.inf for r,y in zip(msg.ranges, points)]
        self.redis_client.set(self.obstacles_back_key, str(min(new_ranges)))

        ### SIDE
        points = [r * np.cos(theta) if (theta > np.pi/4) else np.inf for r,theta in zip(msg.ranges, angles)]
        new_ranges = [r if abs(x) < 0.5 else np.inf for r,x in zip(msg.ranges, points)]
        self.redis_client.set(self.obstacles_side_key, str(min(new_ranges)))

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

    def publish_music_base(self, msg: Twist):
        linear = msg.linear.x
        if abs(linear) > 1e-3:
            self.redis_client.set("music::0", "play")
        else:
            self.redis_client.set("music::0", "stop")

        angular = msg.angular.z
        if abs(angular) > 1e-3:
            self.redis_client.set("music::1", "play")
        else:
            self.redis_client.set("music::1", "stop")

    def publish_music_joints(self, msg: JointState):
        if "joint_lift" in msg.name:
            lift_index = msg.name.index("joint_lift")
            lift_vel = msg.velocity[lift_index]
            if abs(lift_vel) > 1e-3:
                self.redis_client.set("music::2", "play")
            else:
                self.redis_client.set("music::2", "stop")

        if "joint_arm_l3" in msg.name:
            arm_index = msg.name.index("joint_arm_l3")
            arm_vel = msg.velocity[arm_index]
            if abs(arm_vel) > 1e-3:
                self.redis_client.set("music::3", "play")
            else:
                self.redis_client.set("music::3", "stop")

        if "joint_wrist_yaw" in msg.name:
            wrist_yaw_index = msg.name.index("joint_wrist_yaw")
            wrist_yaw_vel = msg.velocity[wrist_yaw_index]
            if abs(wrist_yaw_vel) > 1e-3:
                self.redis_client.set("music::4", "play")
            else:
                self.redis_client.set("music::4", "stop")

        if "joint_wrist_pitch" in msg.name:
            wrist_pitch_index = msg.name.index("joint_wrist_pitch")
            wrist_pitch_vel = msg.velocity[wrist_pitch_index]
            if abs(wrist_pitch_vel) > 1e-3:
                self.redis_client.set("music::5", "play")
            else:
                self.redis_client.set("music::5", "stop")

        
