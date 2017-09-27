#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import imutils
import cv2
import math

do_visualization_img_proc = True
do_visualization_final_result = True

# detection parameters
greenLower = (30, 70, 0)
greenUpper = (45, 255, 255)

# dialation_radius_correction = 0.5

min_radius = 8 # pixels

pixel_size = 3.75 * 10**(-6) # in m
focal_length = 8 * 10**(-3) # in m
tennis_ball_size = 6.54 * 10**(-2) # in m
image_width = 1288
image_height = 964

class Ball_tracking:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_color", Image, self.callback, queue_size=1)
        self.ball_pub = rospy.Publisher("/tennisball", Float64MultiArray, queue_size=1)
        self.image = None
        self.num_iterations = 0

        self.ts_curr = 0
        self.ts_prev = 0

    def callback(self, data):
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.initialized = True
        except CvBridgeError as e:
            print(e)

    def process(self):
        self.ts_prev = self.ts_curr
        self.ts_curr = rospy.get_time()
        self.dt = self.ts_curr - self.ts_prev;None, None, None

        if self.image == None:
            rospy.logwarn("Waiting for image...")
            return None, None, None, None

        image = self.image.copy()

        if not self.compute_image(image):
            return None, None, None, None

        X, Y, Z, bearing = self.locate_3d(self.x, self.y, self.r) # get 3D coordinate in camera frame

        rospy.loginfo("Tracking: X=%.2f Y=%.2f Z=%.2f bearing=%.2f" % (X, Y, Z, bearing * 180 / math.pi))

        self.num_iterations += 1

        if do_visualization_final_result:
            cv2.circle(image, (int(self.x), int(self.y)), int(self.r), (0, 0, 255), 2) # visualization
            text = "Position=(%.3f,%.3f,%.3f) Bearing=%.3f" % (X, Y, Z, bearing * 180 / math.pi)
            cv2.putText(image, text, (10, 500), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0))
            cv2.imshow("Final Results", image)
            cv2.waitKey(3)

        return X, Y, Z, bearing

    def locate_3d(self, x, y, r):
        z_ball = (tennis_ball_size * focal_length) / (r * 2 * pixel_size)
        x_ball = z_ball * (x - image_width / 2.0) * pixel_size / focal_length
        y_ball = z_ball * (y - image_height / 2.0) * pixel_size / focal_length
        bearing = math.atan(x_ball / z_ball)

        return x_ball, y_ball, z_ball, bearing

    def compute_image(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        kernel = np.ones((2,2),np.uint8)
        # mask = cv2.erode(mask, kernel, iterations=2)
        # mask = cv2.dilate(mask, kernel, iterations=1)

        #use colored image for visualization
        if do_visualization_img_proc:
            mask_clr = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)

        if do_visualization_img_proc:
            cv2.imshow("After filtering", mask_clr)
            cv2.imshow("Input Image", image)
            cv2.waitKey(3)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        contours = sorted(contours, key=lambda c: cv2.contourArea(c), reverse=True)

        if len(contours) <= 0:
            rospy.logwarn("No valid contours.")
            return False

        if cv2.contourArea(contours[0]) > 100000:
            rospy.logwarn("Contour area too large.")
            return False

        ((self.x, self.y), self.r) = cv2.minEnclosingCircle(contours[0])
        if self.r < min_radius:
            rospy.logwarn("Enclosed circle is less than minimum radius.")
            return False

        return True

def main():
    node = Ball_tracking()
    rospy.init_node('ball_tracking', anonymous=True)

    rate = rospy.Rate(4)
    while not rospy.is_shutdown():
        X, Y, Z, bearing = node.process()

        flt = Float64MultiArray()

        if not X or not Y or not Z or not bearing:
            flt.data = [0, 0, 0, 0, 0]
            node.ball_pub.publish(flt)
        else:
            flt.data = [1, X, Y, Z, bearing]
            node.ball_pub.publish(flt)

        rate.sleep()
    rospy.spin()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()