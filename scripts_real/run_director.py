import rclpy

from flocking.ros2 import Director


def main(args=None):
    rclpy.init(args=args)
    director = Director(robots=(1,2))
    rclpy.spin(director)
    detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
