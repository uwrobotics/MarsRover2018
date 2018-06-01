#!/usr/bin/env python
import rospy
import roslib
import struct
import sys

from sensor_msgs.msg import Joy
from can_msgs.msg import Frame

class ArmController:
    # Constructor
    def __init__(self, sub_topic='/joy', pub_topic='/CAN_transmitter'):
        self._node = rospy.init_node('arm_control', anonymous=True)
        self._joySub = rospy.Subscriber(sub_topic, Joy, self.joyCallback)
        self._canPub = rospy.Publisher(pub_topic, Frame, queue_size=20)
        self._id = 400 # initial CAN id
        self._pwm = 0
        self._pressed = False

    # Joystick ROS callback
    def joyCallback(self, joy):
        if(joy.buttons[0] == 1 and not self._pressed):
            self._id += 1
            if(self._id > 405):
                self._id = 400
            self._pressed = True
        if(not joy.buttons[0]):
            self._pressed = False
        self._pwm = joy.axes[1]
        self.sendAngle()

    # Convert angles to CAN frame msg type
    def angleToFrame(self):
        frame = Frame()
        frame.id = self._id
        frame.is_rtr = False
        frame.is_extended = False
        frame.is_error = False
        frame.dlc = 4
        data = bytearray(struct.pack('f', self._pwm))
        frame.data = str(data)
        return frame

    # Publish angles to SocketCAN
    def sendAngle(self):
        canFrame = self.angleToFrame()
        self._canPub.publish(canFrame)


if __name__ == '__main__':
    controller = ArmController()
    rospy.spin()
