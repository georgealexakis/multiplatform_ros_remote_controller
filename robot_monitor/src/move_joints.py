#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_msgs.msg import Header


class JointsMover:
    # Initial positions
    pose1 = 0.0
    pose2 = 0.0

    def __init__(self):
        # Init node
        rospy.init_node('move_joints', anonymous=True)
        # Init publisher and subscriber
        self.pub_move = rospy.Publisher(
            '/move_joints', JointState, queue_size=1)
        # Rotation direction with min value: -3.142, max value: +3.142
        rospy.Subscriber('/move_rads_1', Float64, self.callback1)
        # Min value (up): -0.6, Max value (down): +0.6
        rospy.Subscriber('/move_rads_2', Float64, self.callback2)
        self.rate = rospy.Rate(100)
        # Let's spin
        rospy.spin()

    def callback1(self, data):
        # New Joint State message 1
        self.pose1 = data.data
        print(self.pose1)
        joints = JointState()
        joints.header = Header()
        joints.header.stamp = rospy.Time.now()
        joints.name = ['ssl_joint', 'sl_joint', 'f_r_wheel_joint',
                       'f_l_wheel_joint', 'b_r_wheel_joint', 'b_l_wheel_joint']
        joints.position = [self.pose1, self.pose2, 0.0, 0.0, 0.0, 0.0]
        joints.velocity = []
        joints.effort = []
        # Let's publish
        self.pub_move.publish(joints)
        self.rate.sleep()

    def callback2(self, data):
        # New Joint State message 2
        self.pose2 = data.data
        print(self.pose2)
        joints = JointState()
        joints.header = Header()
        joints.header.stamp = rospy.Time.now()
        joints.name = ['ssl_joint', 'sl_joint', 'f_r_wheel_joint',
                       'f_l_wheel_joint', 'b_r_wheel_joint', 'b_l_wheel_joint']
        joints.position = [self.pose1, self.pose2, 0.0, 0.0, 0.0, 0.0]
        joints.velocity = []
        joints.effort = []
        # Let's publish
        self.pub_move.publish(joints)
        self.rate.sleep()


def main():
    JointsMover()


if __name__ == "__main__":
    main()
