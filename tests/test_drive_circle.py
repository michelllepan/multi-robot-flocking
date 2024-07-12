from functools import partial

import numpy as np
import rospy

from flocking.ros.robot import Robot


def main():
    rospy.init_node('move_to')
    robot_0 = Robot(robot_id=0)
    robot_1 = Robot(robot_id=1)
    rate = rospy.Rate(10.0)

    print("setting goal")

    robot_0.set_goal(heading=np.pi/2)
    robot_1.set_goal(heading=-np.pi/2)
    
    while not rospy.is_shutdown():
        robot_0.update_odom()
        robot_1.update_odom()

        print(robot_0.check_at_heading_goal())
        print(robot_1.check_at_heading_goal())

        if not robot_0.check_at_heading_goal():
            # print("robot 0 turning")
            robot_0.turn_toward_goal()
        if not robot_1.check_at_heading_goal():
            # print("robot 1 turning")
            robot_1.turn_toward_goal()

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')