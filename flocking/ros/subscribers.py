import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


class JointStateSubscriber:

    def __init__(self, robot_id):
        self.data = None
        self.sub = rospy.Subscriber(
            name=f'/robot_{robot_id}/joint_states',
            data_class=JointState,
            callback=self.callback)

    def callback(self, data):
        self.data = data


class OdomSubscriber:

    def __init__(self, robot_id):
        self.data = None
        self.position = None
        self.orientation = None
        self.sub = rospy.Subscriber(
            name=f'/robot_{robot_id}/stretch_diff_drive_controller/odom',
            data_class=Odometry,
            callback=self.callback)

    def callback(self, data):
        self.data = data
        self.position = data.pose.pose.position
        self.orientation = data.pose.pose.orientation