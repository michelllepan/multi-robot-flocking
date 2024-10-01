import numpy as np
import redis
import time

REDIS_HOST = "localhost"
REDIS_PORT = "6379"
redis_client = redis.Redis(host=REDIS_HOST, port=REDIS_PORT)
def oscillate():
    lift = 0.3
    arm = 0.0
    wrist_yaw = 0.0
    wrist_pitch = 0.0

    while True:

        lift = 0.6 + 0.3 * np.sin(time.time() * np.pi * 2 / 45)
        arm = 0.015 + 0.015 * np.sin(time.time() * np.pi * 2 / 15 + 3)
        wrist_yaw = 2.0 + 1.0 * np.sin(time.time() * np.pi * 2 / 15)
        wrist_pitch = -0.5 + 0.5 * np.sin(time.time() * np.pi * 2 / 15)

        arm_dict = {
            "lift": (lift, 0.02),
            "arm_l3": (arm, 0.01),
            "arm_l2": (arm, 0.01),
            "arm_l1": (arm, 0.01),
            "arm_l0": (arm, 0.01),
            "wrist_yaw": (wrist_yaw, 0.2),
            "wrist_pitch": (wrist_pitch, 0.2),
        }
        redis_client.set("robot_2::arm", str(arm_dict))
        redis_client.set("robot_1::arm", str(arm_dict))
        time.sleep(0.1)

def oscillate_smaller():
    while True:

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
        redis_client.set("robot_2::arm", str(arm_dict))

        lift = 0.7 + 0.15 * np.sin(time.time() * np.pi * 2 / 10 + 8)
        arm = 0.015 + 0.015 * np.sin(time.time() * np.pi * 2 / 9 + 3)
        wrist_yaw = 2.0 + 1.0 * np.sin(time.time() * np.pi * 2 / 12)
        wrist_pitch = -0.5 + 0.5 * np.sin(time.time() * np.pi * 2 / 7)
        wrist_roll = 1.0 * np.sin(time.time() * np.pi * 2 / 20)

        # lift = 0.5 + 0.3 * np.sin(time.time() * np.pi * 2 / 10 + 8)
        # arm = 0.015 + 0.015 * np.sin(time.time() * np.pi * 2 / 9 + 3)
        # wrist_yaw = 2.0 + 1.0 * np.sin(time.time() * np.pi * 2 / 12)
        # wrist_pitch = -0.5 + 0.5 * np.sin(time.time() * np.pi * 2 / 7)
        # wrist_roll = 1.0 * np.sin(time.time() * np.pi * 2 / 20)

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

        # arm_dict = {
        #     "lift": (lift, 0.1),
        #     "arm_l3": (arm, 0.1),
        #     "arm_l2": (arm, 0.1),
        #     "arm_l1": (arm, 0.1),
        #     "arm_l0": (arm, 0.1),
        #     "wrist_yaw": (wrist_yaw, 0.7),
        #     "wrist_pitch": (wrist_pitch, 0.7),
        #     "wrist_roll": (wrist_roll, 0.7),
        # }

        redis_client.set("robot_1::arm", str(arm_dict))
        redis_client.set("robot_3::arm", str(arm_dict))
        redis_client.set("robot_4::arm", str(arm_dict))
        time.sleep(0.1)

pos_1 = {
    "lift": 0.9,
    "arm_l3": 0.0,
    "arm_l2": 0.0,
    "arm_l1": 0.0,
    "arm_l0": 0.0,
    "wrist_yaw": 3.0,
    "wrist_pitch": -0.5,
}

pos_2 = {
    "lift": 0.5,
    "arm_l3": 0.01,
    "arm_l2": 0.01,
    "arm_l1": 0.01,
    "arm_l0": 0.01,
    "wrist_yaw": 1.0,
    "wrist_pitch": -1.0,
}

arm = 0.015 + 0.015 * np.sin(time.time() * np.pi * 2 / 9 + 3)
wrist_yaw = 2.0 + 1.0 * np.sin(time.time() * np.pi * 2 / 12)
wrist_pitch = -0.5 + 0.5 * np.sin(time.time() * np.pi * 2 / 7)
wrist_roll = 1.0 * np.sin(time.time() * np.pi * 2 / 20)

# furthest extension
pos_3 = {
    "lift": 0.3,
    "arm_l3": 0.03,
    "arm_l2": 0.03,
    "arm_l1": 0.03,
    "arm_l0": 0.03,
    "wrist_yaw": 1.0,
    "wrist_pitch": 0.0,
}

# retracted
pos_4 = {
    "lift": 0.5,
    "arm_l3": 0.0,
    "arm_l2": 0.0,
    "arm_l1": 0.0,
    "arm_l0": 0.0,
    "wrist_yaw": 3.0,
    "wrist_pitch": -1.0,
}

def go_to():
    speed = 0.2
    pos = pos_4
    arm_dict = {
        "lift": (pos["lift"], speed),
        "arm_l3": (pos["arm_l3"], speed),
        "arm_l2": (pos["arm_l2"], speed),
        "arm_l1": (pos["arm_l1"], speed),
        "arm_l0": (pos["arm_l0"], speed),
        "wrist_yaw": (pos["wrist_yaw"], 3 * speed),
        "wrist_pitch": (pos["wrist_pitch"], 3 * speed),
        "wrist_roll": (1.5, 3 * speed),
        "gripper_finger_left": (0.0, speed),
        "gripper_finger_right": (0.0, speed),
    }
    redis_client.set("robot_2::arm", str(arm_dict))

if __name__ == "__main__":
    oscillate_smaller()
    # go_to()