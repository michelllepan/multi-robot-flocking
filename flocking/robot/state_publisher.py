import os
import time
from io import BytesIO

import cv2
import numpy as np
import redis
from PIL import Image
from scipy.spatial.transform import Rotation
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState, JointState, LaserScan
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from flocking.humans import RealSenseCamera
from flocking.humans.utils import get_depth_at_pixel
from flocking.utils import Pose


class StatePublisher(Node):

    def __init__(self, robot_id, redis_host, redis_port):
        super().__init__("state_publisher")

        robot_name = "robot_" + str(robot_id)
        self.get_logger().info("creating state publisher")

        # redis
        self.redis_client = redis.Redis(host=redis_host, port=redis_port)
        self.pose_key = robot_name + "::pose"
        self.battery_key = robot_name + "::battery"
        self.obstacles_front_key = robot_name + "::obstacles::front"
        self.obstacles_back_key = robot_name + "::obstacles::back"
        self.obstacles_side_key = robot_name + "::obstacles::side"
        self.humans_key = robot_name + "::humans"
        self.color_image_key = robot_name + "::color_image"
        self.depth_image_key = robot_name + "::depth_image" 
        self.head_key = robot_name + "::head"
        self.music_key_prefix = robot_name + "::music::"

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

        # head
        self.joint_states_sub = self.create_subscription(
            JointState, '/joint_states', self.publish_head, 1)
        self.head = None

        # person detection
        self.camera = RealSenseCamera(width=640, height=360)
        #self.camera_timer = self.create_timer(1/15, self.publish_image)
        self.yolo = YOLO("models/yolov5nu.pt")
        self.yolo_timer = self.create_timer(0.1, self.publish_humans)
        self.yolo_threshold = 0.5

        # music
        self.music_base_sub = self.create_subscription(
            Twist, '/stretch/cmd_vel', self.publish_music_base, 1)
        self.music_joints_sub = self.create_subscription(
            JointState, '/joint_states', self.publish_music_joints, 1)

        self.get_logger().info("created state publisher")

    def publish_pose(self):
        try:
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
        except redis.exceptions.ConnectionError as e:
            print(e, f" at time {time.time() : .0f}")

    def publish_battery(self, msg: BatteryState):
        try:
            voltage = msg.voltage
            self.redis_client.set(self.battery_key, voltage)
        except redis.exceptions.ConnectionError as e:
            print(e, f" at time {time.time() : .0f}")

    def publish_obstacles(self, msg: LaserScan):
        try:
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
        except redis.exceptions.ConnectionError as e:
            print(e, f" at time {time.time() : .0f}")

    # def publish_humans(self, detections_robot):
    #     if self.pose is None or self.head is None or detections_robot is None:
    #         return
    #     elif len(detections_robot) == 0:
    #         self.redis_client.set(self.humans_key, str(detections_robot))
    #         return

    #     detections_robot = np.array(detections_robot)
    #     if detections_robot.ndim < 2:
    #         detections_robot = np.expand_dims(detections_robot, axis=0)

    #     # add z coordinate for rotation
    #     z = np.zeros((detections_robot.shape[0], 1))
    #     detections_robot = np.hstack((detections_robot, z))

    #     # rotation
    #     rot = Rotation.from_euler("z", self.pose.h + self.head)
    #     detections_world = rot.apply(detections_robot)
    #     detections_world = detections_world[:, :2]

    #     # translation
    #     detections_world += np.array([self.pose.x, self.pose.y])

    #     detections_world = detections_world.tolist()
    #     self.redis_client.set(self.humans_key, str(detections_world))

    def publish_humans(self):
        try:
            if self.pose is None or self.head is None:
                return
            
            depth_frame, color_frame = self.camera.get_frames()
            depth_image, color_image = self.camera.convert_to_array(depth_frame, color_frame)
            height, width, c = color_image.shape

            results = self.yolo([cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)])

            cls = results[0].boxes.cls
            conf = results[0].boxes.conf
            xyxy = results[0].boxes.xyxyn

            detections_robot = []
            for i in range(len(cls)):
                if cls[i] != 0 or conf[i] < self.yolo_threshold: # class 0 is "person"
                    continue

                x_img = (xyxy[i][0] + xyxy[i][2]) / 2
                y_img = (xyxy[i][1] + xyxy[i][3]) / 2

                depth = get_depth_at_pixel(depth_image, x_img, y_img)
                if depth is None:
                    continue

                x_robot = depth
                y_robot = width * (0.5 - x_img) * (depth / 480)
                detections_robot.append((x_robot, y_robot))

            if len(detections_robot) == 0:
                self.redis_client.set(self.humans_key, str(detections_robot))
                return
            
            detections_robot = np.array(detections_robot)

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

        except redis.exceptions.ConnectionError as e:
            print(e, f" at time {time.time() : .0f}")

    def publish_image(self):
        try:
            # start = time.time()
            depth_frame, color_frame = self.camera.get_frames()
            # depth_image, color_image = self.camera.convert_to_array(depth_frame, color_frame)
            color_image = np.asanyarray(color_frame.get_data())
            color_image = np.rot90(color_image, 3)
            # color_image = cv2.resize(color_image, dsize=(360, 180), interpolation=cv2.INTER_CUBIC)

            color_image = cv2.cvtColor(color_image, cv2.COLOR_BGRA2RGB)
            _, jpeg_encoded = cv2.imencode(".jpg", color_image)
            self.redis_client.set(self.color_image_key, jpeg_encoded.tobytes())
            # end = time.time()

            # print("stream image rate:", 1 / (end - start))
            

            # color_output = BytesIO()
            # color_image = color_image[:,:,::-1] # switch order of color channels
            # color_image = Image.fromarray(color_image.astype("uint8"), "RGB")
            # color_image.save(color_output, format="png")
            # self.redis_client.set(self.color_image_key, color_output.getvalue())
            # color_output.close()

            # depth_output = BytesIO()
            # np.save(depth_output, depth_image, allow_pickle=False)
            # self.redis_client.set(self.depth_image_key, depth_output.getvalue())
            # depth_output.close()
        except redis.exceptions.ConnectionError as e:
            print(e, f" at time {time.time() : .0f}")

    def publish_head(self, msg: JointState):
        try:
            index = msg.name.index("joint_head_pan")
            value = msg.position[index]
            self.head = value
            self.redis_client.set(self.head_key, self.head)
        except redis.exceptions.ConnectionError as e:
            print(e, f" at time {time.time() : .0f}")

    def publish_music_base(self, msg: Twist):
        try:
            linear = msg.linear.x
            if abs(linear) > 1e-3:
                self.redis_client.set(self.music_key_prefix + "0", "play")
            else:
                self.redis_client.set(self.music_key_prefix + "0", "stop")

            angular = msg.angular.z
            if abs(angular) > 1e-3:
                self.redis_client.set(self.music_key_prefix + "1", "play")
            else:
                self.redis_client.set(self.music_key_prefix + "1", "stop")
        except redis.exceptions.ConnectionError as e:
            print(e, f" at time {time.time() : .0f}")

    def publish_music_joints(self, msg: JointState):
        try:
            if "joint_lift" in msg.name:
                lift_index = msg.name.index("joint_lift")
                lift_vel = msg.velocity[lift_index]
                if abs(lift_vel) > 1e-3:
                    self.redis_client.set(self.music_key_prefix + "2", "play")
                else:
                    self.redis_client.set(self.music_key_prefix + "2", "stop")

            if "joint_arm_l3" in msg.name:
                arm_index = msg.name.index("joint_arm_l3")
                arm_vel = msg.velocity[arm_index]
                if abs(arm_vel) > 1e-3:
                    self.redis_client.set(self.music_key_prefix + "3", "play")
                else:
                    self.redis_client.set(self.music_key_prefix + "3", "stop")

            if "joint_wrist_yaw" in msg.name:
                wrist_yaw_index = msg.name.index("joint_wrist_yaw")
                wrist_yaw_vel = msg.velocity[wrist_yaw_index]
                if abs(wrist_yaw_vel) > 1e-3:
                    self.redis_client.set(self.music_key_prefix + "4", "play")
                else:
                    self.redis_client.set(self.music_key_prefix + "4", "stop")

            if "joint_head_pan" in msg.name:
                head_pan_index = msg.name.index("joint_head_pan")
                head_pan_vel = msg.velocity[head_pan_index]
                if abs(head_pan_vel) > 1e-3:
                    self.redis_client.set(self.music_key_prefix + "5", "play")
                else:
                    self.redis_client.set(self.music_key_prefix + "5", "stop")

            if "joint_gripper_finger_left" in msg.name:
                gripper_finger_index = msg.name.index("joint_gripper_finger_left")
                gripper_finger_vel = msg.velocity[gripper_finger_index]
                if abs(gripper_finger_vel) > 1e-3:
                    self.redis_client.set(self.music_key_prefix + "6", "play")
                else:
                    self.redis_client.set(self.music_key_prefix + "6", "stop")
        except redis.exceptions.ConnectionError as e:
            print(e, f" at time {time.time() : .0f}")

            
