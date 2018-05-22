#!/usr/bin/env python
from sensor_msgs.msg import Joy
from can_msgs.msg import Frame
import rospy
import struct


class GimbalController:

    def __init__(self):

        self.init_camera = rospy.init_node("Gimbal", anonymous=True)
        self.sub_camera = rospy.Subscriber("/joy", Joy, self.callback)
        self.pub_camera = rospy.Publisher("/gimbal_camera", Frame,
                                          queue_size=20)
        self.pan = 0
        self.tilt = 0
        self.ID = 600

    def callback(self, joy):

        shift = rospy.get_param('move')

        if joy.axes[5] == 1:
            self.tilt += shift

        elif joy.axes[5] == -1:
            self.tilt -= shift

        elif joy.axes[4] == -1:
            self.pan += shift

        elif joy.axes[4] == 1:
            self.pan -= shift

        self.tilt = min(max(self.tilt, -60), 60)

        self.pan = min(max(self.pan, -180), 180)     # do at very end

        self.sendAngle()


# Convert angles to CAN frame msg type
    def angleToFrame(self):
        frame = Frame()
        frame.id = self.ID
        frame.is_rtr = False
        frame.is_extended = False
        frame.is_error = False
        frame.dlc = 8
        data = bytearray(struct.pack('i', self.pan))
        data.extend(bytearray(struct.pack('i', self.tilt)))
        frame.data = str(data)
        return frame

    # Publish angles to SocketCAN
    def sendAngle(self):
        canFrame = self.angleToFrame()
        self.pub_camera.publish(canFrame)

if __name__ == '__main__':
    controller = GimbalController()
    rospy.spin()
