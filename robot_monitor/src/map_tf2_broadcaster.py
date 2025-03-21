#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg


class FixedTFBroadcaster:

    def __init__(self):
        self.pub_tf = rospy.Publisher(
            '/tf', tf2_msgs.msg.TFMessage, queue_size=1)

        while not rospy.is_shutdown():
            # Run this loop at about 10Hz
            rospy.sleep(0.1)
            # Connect robot tf with camera tf
            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = 'map'
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = 't265_odom_frame'
            # Transformation - Put camera in the map and set it above the surface of the grid
            t.transform.translation.x = 0.2
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.55
            # Rotation
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            tfm = tf2_msgs.msg.TFMessage([t])
            self.pub_tf.publish(tfm)


if __name__ == '__main__':
    rospy.init_node('fixed_tf2_broadcaster')
    tfb = FixedTFBroadcaster()

    rospy.spin()
