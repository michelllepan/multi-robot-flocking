import numpy as np
import redis
import time


REDIS_HOST = "localhost"
REDIS_PORT = "6379"


class ArmMover:

    def __init__(self, robots=(1,)):
        super().__init__()
        self.redis_client = redis.Redis(host=REDIS_HOST, port=REDIS_PORT)
        # TODO: check robots

    def send_arm_commands(self):
        flock_state_str = self.redis_client.get("state")
        flock_state = flock_state_str.decode("utf-8") if flock_state_str else "STOP"

        if flock_state == "GESTURE_CLAP":
            self.clap()
        else:
            self.oscillate()

    def clap(self):
        gripper_open = int(time.time()) % 2 == 0
        # print(gripper_open)
        # gripper_open = False
        arm_dict = {
            "lift": (0.5, 0.03),
            "gripper_finger_left": (0.2 if gripper_open else 0.0, 30.0),
            # "gripper_finger_right": (0.5 if gripper_open else 0.0, 0.5),
            # "gripper_aperture": (0.5 if gripper_open else 0.0, 0.5),
        }
        self.redis_client.set("robot_1::arm", str(arm_dict))
        self.redis_client.set("robot_2::arm", str(arm_dict))
        self.redis_client.set("robot_3::arm", str(arm_dict))
        self.redis_client.set("robot_4::arm", str(arm_dict))

    def oscillate(self):
        lift = 0.7 + 0.15 * np.sin(time.time() * np.pi * 2 / 10 + 8)
        arm = 0.015 + 0.015 * np.sin(time.time() * np.pi * 2 / 9 + 3)
        wrist_yaw = 2.0 + 1.0 * np.sin(time.time() * np.pi * 2 / 12)
        wrist_pitch = -0.5 + 0.5 * np.sin(time.time() * np.pi * 2 / 7)
        wrist_roll = 1.0 * np.sin(time.time() * np.pi * 2 / 20)

        arm_dict = {
            "lift": (lift, 0.03),
            "arm_l3": (arm, 0.02),
            "arm_l2": (arm, 0.02),
            "arm_l1": (arm, 0.02),
            "arm_l0": (arm, 0.02),
            "wrist_yaw": (wrist_yaw, 0.3),
            "wrist_pitch": (wrist_pitch, 0.2),
            "wrist_roll": (wrist_roll, 0.4),
            "gripper_finger_left": (0.0, 0.2),
        }

        self.redis_client.set("robot_1::arm", str(arm_dict))
        self.redis_client.set("robot_3::arm", str(arm_dict))

        lift = 0.5 + 0.2 * np.sin(time.time() * np.pi * 2 / 15 + 8)
        arm = 0.015 + 0.015 * np.sin(time.time() * np.pi * 2 / 15 + 3)
        wrist_yaw = 2.0 + 1.0 * np.sin(time.time() * np.pi * 2 / 15)
        wrist_pitch = -0.5 + 0.5 * np.sin(time.time() * np.pi * 2 / 15)
        wrist_roll = 1.0 * np.sin(time.time() * np.pi * 2 / 20)

        arm_dict = {
            "lift": (lift, 0.02),
            "arm_l3": (arm, 0.01),
            "arm_l2": (arm, 0.01),
            "arm_l1": (arm, 0.01),
            "arm_l0": (arm, 0.01),
            "wrist_yaw": (wrist_yaw, 0.2),
            "wrist_pitch": (wrist_pitch, 0.2),
            "wrist_roll": (wrist_roll, 0.2),
            "gripper_finger_left": (0.0, 0.2),
        }
        self.redis_client.set("robot_2::arm", str(arm_dict))
        self.redis_client.set("robot_4::arm", str(arm_dict))