import argparse

import rclpy

from flocking.ros2 import Director


def main(args=None, robots=(1,)):
    rclpy.init(args=args)
    director = Director(robots=robots)
    rclpy.spin(director)
    detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--robots", "-r", nargs="+", type=int, required=True)
    args = parser.parse_args()
    main(robots=tuple(args.robots))
