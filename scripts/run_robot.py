import os
import time
from multiprocessing import Process

import rclpy

from flocking.robot import FlockFollower, MusicPlayer


# def run_follower(robot_id):
#     rclpy.init()
#     robot = FlockFollower(robot_id)
#     rclpy.spin(robot)
#     robot.destroy_node()
#     rclpy.shutdown()

# def run_music():
#     player = MusicPlayer()
#     while True:
#         player.update()
#         time.sleep(0.1)

# def main(robot_id):
#     p1 = Process(target=run_follower, args=(robot_id,))
#     p1.start()

#     p2 = Process(target=run_music)
#     p2.start()

#     p1.join()
#     p2.join()

def main(args=None, robot_id=1):
    rclpy.init(args=args)
    robot = FlockFollower(robot_id)
    rclpy.spin(robot)
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    robot_id = os.environ.get("ROBOT_ID", 1)
    main(robot_id=robot_id)
