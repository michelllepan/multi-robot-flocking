import argparse

import numpy as np
import rospy

from flocking.boids import BoidsRunner
from flocking.ros.robot import Robot


MODE = "LINEAR_TRACKS"
TARGET_COLOR = "mediumpurple"
ROBOT_COLOR = "steelblue"


def main(num_robots: int = 3, mode: str = "DEFAULT"):
    rospy.init_node('boids_circle')
    robots = [Robot(robot_id=i) for i in range(num_robots)]
    for r in robots:
        r.update_odom()

    robot_starts = np.array([[r.x, r.y] for r in robots])
    boids_runner = BoidsRunner(
        num_robots=num_robots,
        robot_start_positions=robot_starts,
        step_scale=0.02,
        canvas_dims=(15, 12))
    rate = rospy.Rate(10.0)

    boids_runner.move_human(np.array([-1, -1]))
    boids_runner.update_targets(steps=50, mode=mode)

    t = 0
    while not rospy.is_shutdown():
        print(f"t = {t}")
        if t % 3 == 0:
            boids_runner.update_targets(steps=50, mode=mode)
            for i, r in enumerate(robots):
                r.set_goal(
                    x=boids_runner.target_positions[i, 0],
                    y=boids_runner.target_positions[i, 1])
                
        for r in robots:
            if not r.check_at_pos_goal():
                r.move_toward_goal()
            r.update_odom()
        
        robot_positions = np.array([[r.x, r.y] for r in robots])
        boids_runner.move_robots(robot_positions)

        t += 1
        rate.sleep()


if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument("--num_robots", "-n", type=int, default=3)
        parser.add_argument("--mode", "-m", default="DEFAULT")
        args = parser.parse_args()
        main(
            num_robots=args.num_robots,
            mode=args.mode,
        )
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')