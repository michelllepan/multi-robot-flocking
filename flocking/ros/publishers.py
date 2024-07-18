import rospy
from geometry_msgs.msg import Point, Twist


class VelocityPublisher:

    def __init__(self, robot_id):
        self.pub = rospy.Publisher(
            f'/robot_{robot_id}/stretch_diff_drive_controller/cmd_vel',
            Twist,
            queue_size=1)
        
    def publish(self, message: Twist):
        self.pub.publish(message)


class TargetPublisher:

    def __init__(self, robot_id):
        self.pub = rospy.Publisher(
            f'/robot_{robot_id}/boids_target',
            Point,
            queue_size=1)
        
    def publish(self, message: Point):
        self.pub.publish(message)


class CarrotPublisher:

    def __init__(self, robot_id):
        self.pub = rospy.Publisher(
            f'/robot_{robot_id}/boids_carrot',
            Point,
            queue_size=1)
        
    def publish(self, message: Point):
        self.pub.publish(message)