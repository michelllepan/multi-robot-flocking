import os

import rclpy

from flocking.robot import StatePublisher


def main(args=None, robot_id=1, redis_host="localhost"):
    rclpy.init(args=args)
    pub = StatePublisher(robot_id, redis_host)
    rclpy.spin(pub)
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    robot_id = os.environ.get("ROBOT_ID", 1)
    redis_host = os.environ.get("REDIS_HOST", "localhost")
    main(robot_id=robot_id, redis_host=redis_host)
