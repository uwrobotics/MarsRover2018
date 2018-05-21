#!/usr/bin/env python
# USAGE
# File name: ball-tracking
# DATE: Feb 19, 2018
# Version: v0.9.0
# UW Robotics Team

# improving lists
# TODO#1: find best fit (such as shape & so on, part in TODO#2) instead of
#         largest size in sorting list
# TODO#2: improve the selection based on the cropped region
# TODO#3: Reduce the noise by confidence level of prev frame (duration
#         without interruption)

# import the necessary packages
from collections import deque
import numpy as np
from collections import namedtuple
import imutils
import cv2

# Output
OUTPUT_Collection = namedtuple("OUTPUT",
                               ["OnScreenX",
                                "OnScreenY",
                                "OnScreenRad",
                                "ScreenW",
                                "ScreenH",
                                "isDetected",
                                "isStableIn20Cycles"])
data_out = OUTPUT_Collection(-1, -1, -1.0, -1, -1, False, False)  # default
# USER Define
DebugOnScreenMode = True
isDetected = False
pts_size = 32
stable_cycles_threshold = 20
greenLower = (25, 0, 0)
greenUpper = (60, 255, 255)
width = 0
height = 0
max_dif_square = 10000
MIN_Radius = 10  # pixel
MAX_Radius = 150  # pixel
TOL_RadiusDiff_BtwnFrames = 100
pts = deque(maxlen=pts_size)
# ----------CODE----------
# grab the reference to the webcam
camera = cv2.VideoCapture(0)
# camera setting, require adjustments
camera.set(cv2.CAP_PROP_APERTURE, 1)
camera.set(cv2.CAP_PROP_AUTOFOCUS, 0)
camera.set(cv2.CAP_PROP_GAIN, 0)
camera.set(cv2.CAP_PROP_EXPOSURE, -2)
camera.set(cv2.CAP_PROP_CONTRAST, 0)
camera.set(cv2.CAP_PROP_BRIGHTNESS, 0)
# keep looping
while True:
    # grab the current frame
    (grabbed, camFrame) = camera.read()

    # resize
    # camFrame = imutils.resize(camFrame)
    camFrame = imutils.resize(camFrame, width=720)
    width = camFrame.shape[1]
    height = camFrame.shape[0]

    # color space
    camFrame_hsv = cv2.cvtColor(camFrame, cv2.COLOR_BGR2HSV)

    # Color Mask
    kernel_clr = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10))
    mask_clr = cv2.inRange(camFrame_hsv, greenLower, greenUpper)

    if(DebugOnScreenMode):
        DebugShow_Mask_before_Filter = cv2.cvtColor(mask_clr,
                                                    cv2.COLOR_GRAY2BGR)

    # filter
    mask_clr = cv2.erode(mask_clr, kernel_clr, iterations=2)
    mask_clr = cv2.dilate(mask_clr, kernel_clr, iterations=2)

    if(DebugOnScreenMode):
        DebugShow_Mask_after_Filter = cv2.cvtColor(mask_clr,
                                                   cv2.COLOR_GRAY2BGR)
        DebugShow_Origin_with_FilteredMask = cv2.bitwise_and(camFrame,
                                                             camFrame,
                                                             mask=mask_clr)

    list_countours = cv2.findContours(mask_clr.copy(),
                                      cv2.RETR_EXTERNAL,
                                      cv2.CHAIN_APPROX_SIMPLE)[-2]

    center = None
    Detected_ballInfo = None
    isDetected = False
    # only proceed if at least one contour was found
    if list_countours is not None and len(list_countours) > 0:
        sorted_contours = sorted(list_countours,
                                 key=cv2.contourArea,
                                 reverse=True)
        # find the largest valid contour in the mask
        # c = max(list_countours, key=cv2.contourArea)
        for c in sorted_contours:
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            # only proceed if the radius is within the ideal radius
            if radius > MIN_Radius and radius < MAX_Radius:
                M = cv2.moments(c)
                if M["m00"] != 0:
                    center = (int(M["m10"] / M["m00"]),
                              int(M["m01"] / M["m00"]))

                    # if pts exist in last frame, let's compare the position
                    if (len(pts) > 0):
                        prev_tennis = pts.popleft()
                        prev_cent = prev_tennis[0]
                        prev_rad = prev_tennis[1]

                        # if its not random noise
                        # TODO#3: Reduce the noise by confidence level of prev
                        #         frame (duration without interruption)
                        if abs(prev_rad - radius) < \
                           TOL_RadiusDiff_BtwnFrames:
                            if (pow((prev_cent[0] - center[0]), 2) +
                                pow((prev_cent[1] - center[1]), 2)) > \
                               max_dif_square:
                                    pts.clear()  # clear
                            else:
                                # TODO#2: improve the selection based on the
                                #         cropped region
                                # crop_x1 = int(x - radius)
                                # crop_x2 = int(x + radius)
                                # if crop_x1 < 0:
                                #     crop_x1 = 0
                                # if crop_x2 > width:
                                #     crop_x2 = width
                                # crop_y1 = int(y - radius)
                                # crop_y2 = int(y + radius)
                                #
                                # if crop_y1 < 0:
                                #     crop_y1 = 0
                                # if crop_y2 > height:
                                #     crop_y2 = height
                                #
                                # print(crop_x1, crop_y1, "|",
                                #       crop_x2, crop_y2)
                                # if (crop_x2 - crop_x1 > 0) and
                                #    (crop_y2 - crop_y1 > 0):
                                #     frame_crop = camFrame[crop_y1:crop_y2,
                                #                           crop_x1:crop_x2]
                                #     # crop_canny = frame_crop.copy()
                                #     # crop_canny = cv2.Canny(crop_canny,
                                #     #                        70,
                                #     #                        200)
                                #     # blur = cv2.cvtColor(frame_crop,
                                #     #                     cv2.COLOR_BGR2GRAY)
                                #     # blur = cv2.medianBlur(blur, 5)
                                #     #
                                #     cv2.imshow("crop", frame_crop)
                                # end of TODO
                                isDetected = True
                                Detected_ballInfo = (center, radius)
                                pts.appendleft(prev_tennis)
                                pts.appendleft(Detected_ballInfo)
                                # draw the circle and centroid on the frame,
                                # then update the list of tracked points
                                if (DebugOnScreenMode):
                                    cv2.circle(camFrame,
                                               (int(x), int(y)),
                                               int(radius),
                                               (0, 255, 255),
                                               2)
                                    cv2.circle(camFrame,
                                               center,
                                               5,
                                               (0, 0, 255),
                                               -1)
                    else:
                        isDetected = True
                        Detected_ballInfo = (center, radius)
                        pts.appendleft(Detected_ballInfo)

                    # we just need a valid one
                    break

    # assign output data
    if(isDetected):
        isStable = bool(len(pts) >= stable_cycles_threshold)
        data_out = OUTPUT_Collection(OnScreenX=Detected_ballInfo[0][0],
                                     OnScreenY=Detected_ballInfo[0][1],
                                     OnScreenRad=Detected_ballInfo[1],
                                     ScreenW=width,
                                     ScreenH=height,
                                     isDetected=True,
                                     isStableIn20Cycles=isStable)
    else:
        data_out = OUTPUT_Collection(-1, -1, -1.0, -1, -1, False, False)

    if (DebugOnScreenMode):
        # show the frame to our screen
        img_mergec1 = np.concatenate((camFrame, DebugShow_Mask_before_Filter),
                                     axis=0)
        img_mergec2 = np.concatenate((DebugShow_Mask_after_Filter,
                                      DebugShow_Origin_with_FilteredMask),
                                     axis=0)
        img_merge = np.concatenate((img_mergec1, img_mergec2), axis=1)
        img_merge = imutils.resize(img_merge, width=1000)
        cv2.imshow("Result", img_merge)
        print(data_out)

        # print(center)

    key = cv2.waitKey(33)

    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break

    if key == ord(" "):
        while True:
            key = cv2.waitKey(0)
            if key == ord("q"):
                break
        break

# cv2.waitKey(0)

# cleanup the camera and close any open windows
camera.release()
cv2.destroyAllWindows()
