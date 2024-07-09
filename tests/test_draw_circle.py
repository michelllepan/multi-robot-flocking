import numpy as np
import rospy
import sys
from rospy.rostime import Time, Duration

import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint


class DrawCircleNode:

    def __init__(self):
        self.trajectory_client = actionlib.SimpleActionClient('/robot_1/stretch_arm_controller/follow_joint_trajectory/', FollowJointTrajectoryAction)
        server_reached = self.trajectory_client.wait_for_server(timeout=rospy.Duration(10.0))
        print(server_reached)
        if not server_reached:
            rospy.signal_shutdown('Unable to connect to arm action server. Timeout exceeded.')
            sys.exit()

    def draw(self):
        arm_init = 0.1
        lift_init = 1.0
        n = 20
        diameter_m = 0.2
        time_dt = 0.75
        globalv_m = None
        globala_m = None

        t = np.linspace(0, 2*np.pi, n, endpoint=True)
        x = (diameter_m/2) * np.cos(t) + arm_init
        y = (diameter_m/2) * np.sin(t) + lift_init
        circle_mat = np.c_[x, y]

        circle_traj = FollowJointTrajectoryGoal()
        circle_traj.trajectory.joint_names = ['joint_wrist_yaw', 'joint_lift']
        for i in range(n):
            pt = circle_mat[i]
            pt_t = i * time_dt
            point = JointTrajectoryPoint()
            point.time_from_start = Duration(secs=pt_t)#.to_msg()
            point.positions = [pt[0], pt[1]]
            circle_traj.trajectory.points.append(point)
        self.trajectory_client.send_goal(circle_traj)

if __name__ == '__main__':
    rospy.init_node('draw_circle')
    circle = DrawCircleNode()
    circle.draw()