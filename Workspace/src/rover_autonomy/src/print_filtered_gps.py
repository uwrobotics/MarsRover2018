#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import *

cnt = 0

def callback(data):
    global cnt
    cnt += 1
    print("%d, %.15f, %.15f" % (cnt, float(data.latitude), float(data.longitude)))
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/gps/filtered", NavSatFix, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
