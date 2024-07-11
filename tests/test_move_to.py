from functools import partial

import rospy

from flocking.ros.robot import Robot


def main():
    rospy.init_node('move_to')
    robot = Robot(robot_id=0)
    rate = rospy.Rate(10.0)
    robot.set_goal(1, 1, 0)
    while not rospy.is_shutdown():
        if robot.check_at_goal(): break
        robot.set_goal(1, 1, 0)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')