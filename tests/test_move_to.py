from functools import partial

import numpy as np
import rospy

from flocking.ros.robot import Robot


def main():
    rospy.init_node('move_to')
    robot_0 = Robot(robot_id=0)
    rate = rospy.Rate(10.0)

    print("setting goal")

    robot_0.set_goal(x=-1, y=0)
    
    while not rospy.is_shutdown():
        robot_0.update_odom()
        robot_0.print_odometry()
        if not robot_0.check_at_pos_goal():
            robot_0.move_toward_goal()

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')