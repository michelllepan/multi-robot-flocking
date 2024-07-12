from functools import partial

import rospy

from flocking.ros.robot import Robot


def main():
    rospy.init_node('move_to')
    robot = Robot(robot_id=0)
    rate = rospy.Rate(10.0)
    robot.set_goal(2, 2)
    while not rospy.is_shutdown():
        robot.update_odom()
        if robot.check_at_goal(): break
        robot.move_towards_goal()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')