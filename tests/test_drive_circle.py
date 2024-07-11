from functools import partial

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState


class DriveCircle:

    def __init__(self, num_robots=1):
        self.rate = 10.0
        self.num_robots = num_robots
        self.cmd_vel_pubs = []
        self.joint_states = []

        rospy.init_node('drive_circle')
        for i in range(self.num_robots):
            self.cmd_vel_pubs.append(
                rospy.Publisher(
                    f'/robot_{i}/stretch_diff_drive_controller/cmd_vel',
                    Twist,
                    queue_size=10))
            self.joint_states.append(None)
            rospy.Subscriber(
                f'/robot_{i}/joint_states',
                JointState,
                partial(self.joint_states_callback, robot_id=i))

    def joint_states_callback(self, joint_state, robot_id):
        self.joint_states[robot_id] = joint_state

    def move_robot(self, robot_id, linear, angular):
        if self.joint_states[robot_id] is None: return

        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular

        pub = self.cmd_vel_pubs[robot_id]
        pub.publish(twist)

    def main(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            for i in range(self.num_robots):
                self.move_robot(i, linear=0.5, angular=0.5)
            rate.sleep()


if __name__ == '__main__':
    try:
        node = DriveCircle(num_robots=2)
        node.main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')