#!/usr/bin/env python
import rospy
import tf
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


class NorthFinder:

    def __init__(self, sub_topic='odometry/filtered',
                 pub_topic='/angle'):
        rospy.init_node('north_finder')
        self.subscriber = rospy.Subscriber(
            sub_topic, Odometry, self.odom_to_yaw)
        self.publisher = rospy.Publisher(pub_topic, Float64, queue_size=1)

    # Converts filtered odometry into a heading angle,
    # and then publishes the angle
    def odom_to_yaw(self, data):
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2] / math.pi * 180
        output_message = Float64()
        output_message.data = yaw
        rospy.logdebug("Publishing yaw: %f", yaw)
        self.publisher.publish(output_message)


if __name__ == '__main__':
    try:
        node = NorthFinder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
