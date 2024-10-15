import numpy as np
import redis
import time


class ArmMover:

    def __init__(self, robots, redis_config):
        super().__init__()
        self.robots = robots
        self.redis_clients = {}

        for i in redis_config:
            host, port = redis_config[i]
            self.redis_clients[i] = redis.Redis(host, port)

    def send_arm_commands(self):
        flock_state_str = self.redis_clients[0].get("state")
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
        for r in self.robots:
            self.redis_clients[r].set(f"robot_{r}::arm", str(arm_dict))

    def oscillate(self):
        lift = 0.7 + 0.15 * np.sin(time.time() * np.pi * 2 / 10 + 8)
        arm = 0.015 + 0.015 * np.sin(time.time() * np.pi * 2 / 9 + 3)
        wrist_yaw = 2.0 + 1.0 * np.sin(time.time() * np.pi * 2 / 12)
        wrist_pitch = -0.5 + 0.5 * np.sin(time.time() * np.pi * 2 / 7)
        wrist_roll = 1.0 * np.sin(time.time() * np.pi * 2 / 20)

        arm_dict_1 = {
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
        arm_dict_3 = arm_dict_1

        lift = 0.5 + 0.2 * np.sin(time.time() * np.pi * 2 / 15 + 8)
        arm = 0.015 + 0.015 * np.sin(time.time() * np.pi * 2 / 15 + 3)
        wrist_yaw = 2.0 + 1.0 * np.sin(time.time() * np.pi * 2 / 15)
        wrist_pitch = -0.5 + 0.5 * np.sin(time.time() * np.pi * 2 / 15)
        wrist_roll = 1.0 * np.sin(time.time() * np.pi * 2 / 20)

        arm_dict_2 = {
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
        arm_dict_4 = arm_dict_2
        
        for r in self.robots:
            self.redis_clients[r].set(f"robot_{r}::arm", str(eval(f"arm_dict_{r}")))