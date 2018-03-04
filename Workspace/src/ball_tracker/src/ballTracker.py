#!/usr/bin/env python
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
    __  ___                   ____
   /  |/  /___ ___________   / __ \____ _   _____  _____
  / /|_/ / __ `/ ___/ ___/  / /_/ / __ \ | / / _ \/ ___/
 / /  / / /_/ / /  (__  )  / _, _/ /_/ / |/ /  __/ /
/_/  /_/\__,_/_/  /____/  /_/ |_|\____./|___/\___/_/

Copyright 2018, UW Robotics Team

@file     BallTracker.py
@author:  Archie Lee

'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''


# improvements:
# TODO#1: find best fit(such as shape & so on, part in TODO#2) insread of largest size in sorting list
# TODO#2: improve the selection based on the cropped region
# TODO#3: Reduce the noise by confidence level of prev frame (duration without interruption)

# ROS
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from ball_tracker.msg import BallDetection

# OpenCV
import cv2

# Other Python packages
from collections import deque
from collections import namedtuple


class BallTracker:
    def __init__(self):
        rospy.init_node('ball_tracker', anonymous=True)
        self.bridge = CvBridge()

        # ROS parameters
        self.image_topic = rospy.get_param('image_topic', 'camera/image_raw')
        self.kernel_dim = rospy.get_param('kernel_dim', 10) # in pixels
        self.colour_upper = rospy.get_param('colour_upper', [60, 255, 255]) # HSV
        self.colour_lower = rospy.get_param('colour_lower', [25, 0, 0]) # HSV
        self.mask_iterations = rospy.get_param('mask_iterations', 2)
        self.max_radius = rospy.get_param('max_radius', 150) # in pixels
        self.min_radius = rospy.get_param('min_radius', 10) # in pixels
        self.max_distance = rospy.get_param('max_distance', 10000) # Euclidean
        self.max_radius_difference = rospy.get_param('max_radius_difference', 10) # in pixels
        self.stability_threshold = rospy.get_param('stability_threshold', 20)
        self.detection_buffer_size = rospy.get_param('detection_buffer_size', 32)

        # Deque to track detection history
        self.prev_detections = deque(maxlen=self.detection_buffer_size)

        # ROS publishers and subscribers
        self.camera_sub = rospy.Subscriber(self.image_topic, Image, self.imageCallback)
        self.detection_pub = rospy.Publisher('ball_detection', BallDetection, queue_size=1)

    def imageCallback(self, image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)

        contours = self.processImage(cv_image)
        detection_msg = self.findDetections(contours)
        self.detection_pub.publish(detection_msg)

    def processImage(self, cv_image):
        # convert image to HSV colourspace
        cv_image_hsv = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV) #TODO: should be configurable depending on image type

        # create colour masks
        kernel_size = (self.kernel_dim, self.kernel_dim)
        kernel_clr = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, kernel_size)
        mask_clr = cv2.inRange(cv_image_hsv, self.colour_lower, self.colour_upper)

        # erode and dilate the image using colour mask
        mask_clr = cv2.erode(mask_clr, kernel_clr, iterations=self.mask_iterations)
        mask_clr = cv2.dilate(mask_clr, kernel_clr, iterations=self.mask_iterations)

        contours = cv2.findContours(mask_clr.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        return contours

    def findDetections(self, contours):
        isDetected = False
        detection = None
        if contours and len(contours) > 0:
            # sort contours by largest area
            sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)
            for contour in sorted_contours:
                ((x, y), radius) = cv2.minEnclosingCircle(contour)
                # continue if radius is in correct range
                if self.min_radius < radius < self.max_radius:
                    moment = cv2.moments(contour)
                    if moment["m00"] != 0:
                        center = (int(moment["m10"] / moment["m00"]), int(moment["m01"] / moment["m00"]))

                        # compare position to last detection
                        if len(self.prev_detections) > 0:
                            prev_detection = self.prev_detections.popleft()
                            prev_cent = prev_detection[0]
                            prev_rad = prev_detection[1]

                            # check for noise
    						# TODO#3: Reduce the noise by confidence level of prev frame (duration without interruption)
                            if abs(prev_rad - radius) < self.max_radius_difference:
                                # Euclidean distance of 2 detection centers
                                dist = pow((prev_cent[0] - center[0]), 2) + pow((prev_cent[1] - center[1]), 2)
                                if (dist > self.max_distance):
                                    self.prev_detections.clear()
                                else:
                                    # TODO#2: improve the selection based on the cropped region
                                    isDetected = True
                                    detection = (center, radius)
                                    self.prev_detections.appendleft(prev_tennis)
                                    self.prev_detections.appendleft(detection)

                        else:
                            isDetected = True
                            detection = (center, radius)
                            self.prev_detections.appendleft(detection)

                        # only care about one valid detection
                        break

        output = BallDetection()

        # assign output data
        if isDetected:
            # track detection stability
            isStable = (len(self.prev_detections) >= self.stability_threshold)
            output.x = detection[0][0]
            output.y = detection[0][1]
            output.rad = detection[1]
            output.isDetected = True
            output.isStable = isStable
        else:
            output.x = -1
            output.y = -1
            output.rad = -1.0
            output.isDetected = False
            output.isStable = isStable

        return output


if __name__ == '__main__':
    ball_tracker = BallTracker()
    rospy.spin()
