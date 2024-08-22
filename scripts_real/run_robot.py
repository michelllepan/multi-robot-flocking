import os

import rclpy

from flocking.ros2 import Robot


def main(args=None, robot_id=1):
    rclpy.init(args=args)
    robot = Robot(robot_id)
    rclpy.spin(robot)
    detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    robot_id = os.environ.get("ROBOT_ID", 1)
    main(robot_id=robot_id)
