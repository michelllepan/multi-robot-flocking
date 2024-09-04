import os

import rclpy

from flocking.robot import StatePublisher


def main(args=None, robot_id=1):
    rclpy.init(args=args)
    pub = StatePublisher(robot_id)
    rclpy.spin(pub)
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    robot_id = os.environ.get("ROBOT_ID", 1)
    main(robot_id=robot_id)
