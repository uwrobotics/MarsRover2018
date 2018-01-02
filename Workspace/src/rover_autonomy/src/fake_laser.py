#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

def main():
    pub = rospy.Publisher('/scan', LaserScan, queue_size=1)
    rospy.init_node('fake_laser', anonymous=True)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():

        rate.sleep()

        scan = LaserScan()

        scan.header.frame_id = "base_link"
        scan.header.stamp = rospy.get_rostime()

        scan.angle_min = -2.3561899662
        scan.angle_max = 2.3561899662

        scan.angle_increment = 0.0065540750511
        scan.time_increment = 0;
        scan.scan_time = 0
        scan.range_min = 0.10000000149
        scan.range_max = 40.0
        scan.ranges = [30] * 720

        pub.publish(scan)

        print(scan.header.stamp)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass