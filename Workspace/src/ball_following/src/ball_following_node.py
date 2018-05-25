#!/usr/bin/env python
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
UWRT Mars Rover Ball Following

Copyright 2018, UW Robotics Team

@file     ball_following.py
@author:  Archie Lee

'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

# ROS
import rospy
from geometry_msgs.msg import Twist
from ball_tracker.msg import BallDetection
from std_msgs.msg import Bool


class Detection:
    def __init__(self, detection_msg):
        self.x = detection_msg.x
        self.y = detection_msg.y
        self.r = detection_msg.radius


class BallFollowing:
    def __init__(self):
        rospy.init_node('ball_following', anonymous=True)

        self.eps = rospy.get_param('radius_epsilon', 0.01)
        self.targetRadius = rospy.get_param('target_radius', 50.0)
        self.maxAngular = rospy.get_param('max_angular', 0.2)
        self.maxLinear = rospy.get_param('max_linear', 0.4)
        self.arrivalThr = rospy.get_param('arrival_threshold', 10)

        tmp = BallDetection(-1.0, -1.0, -1.0, False, False)
        self.lastDetection = Detection(tmp)
        self.arrivalCount = 0

        # ROS subscribers and publishers
        self.detectionSub = rospy.Subscriber('ball_tracker/detection',
                                             BallDetection,
                                             self.detectionCallback)
        rospy.loginfo("BallFollowing: Subscribed to /ball_tracker/detection")
        self.twistPub = rospy.Publisher('ball_following/cmd_vel',
                                        Twist,
                                        queue_size=1)
        rospy.loginfo("BallFollowing: Publishing to /ball_following/cmd_vel")
        self.arrivalPub = rospy.Publisher('ball_following/arrival',
                                          Bool,
                                          queue_size=1)
        rospy.loginfo("BallFollowing: Publishing to /ball_following/arrival")

    def reset(self):
        tmp = BallDetection(-1.0, -1.0, -1.0, False, False)
        self.lastDetection = Detection(tmp)
        self.arrivalCount = 0

    def detectionCallback(self, msg):
        if msg.isDetected and msg.isStable:
            detection = Detection(msg)
            twistOutput = self.calculateTwistFromDetection(detection)
            self.twistPub.publish(twistOutput)

    def calculateTwistFromDetection(self, detection):
        cmd = Twist()
        arrived = False

        # we want to move forward as long as the ball is smaller
        # than a golden threshold
        if detection.r < self.targetRadius:
            cmd.linear.x = self.maxLinear  # make this scale based on r
        else:
            cmd.linear.x = 0.0
            self.arrivalCount += 1
            if self.arrivalCount >= self.arrivalThr:
                arrived = True

        # x is normalized between 0 and 1.0
        # we want to keep the ball in the centre of the frame
        if -self.eps < cmd.linear.x < self.eps:
            cmd.angular.z = 0.0
        elif detection.x > 0.5 + self.eps:
            cmd.angular.z = (detection.x - 0.5) / 0.5 * self.maxAngular
        elif detection.x < 0.5 + self.eps:
            cmd.angular.z = (0.5 - detection.x) / 0.5 * self.maxAngular
        else:
            cmd.angular.z = 0.0

        msg = Bool(arrived)
        self.arrivalPub.publish(msg)

        return cmd


if __name__ == "__main__":
    follower = BallFollowing()
    rospy.spin()
