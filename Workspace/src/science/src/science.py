#!/usr/bin/env python
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt8
from can_msgs.msg import Frame
import rospy
import struct


class ScienceController:
    """
A to open flap, B to close flap. One joy stick to control the spin and
the other joy stick to control the vertical movement.
Author: Zameer Bharwani
"""

    def __init__(self):

        self.init_Auger = rospy.init_node("ScienceController", anonymous=True)
        self.sub_switches = rospy.Subscriber("/science_limit_switches", UInt8,
                                             self.callback_switches)
        self.sub_xbox = rospy.Subscriber("/joy", Joy, self.callback_xbox)
        self.pub = rospy.Publisher("/science", Frame, queue_size=20)
        self.upper_switch = 0
        self.lower_switch = 0
        self.speed_dir = 0
        self.speed_dir_spin = 0
        self.position = 0
        self.ID_limit = 600
        self.ID_spin = 700
        self.ID_flap = 800

    def callback_switches(self, switches):

        if (switches.data) & 0x1 == 1:
            # check to see if the upper switch is being pressed
            self.upper_switch = 1

        else:
            self.upper_switch = 0

        if (switches.data >> 1) & 0x1 == 1:
            # check to see if the lower switch is being pressed
            self.lower_switch = 1

        else:
            self.lower_switch = 0

    def callback_xbox(self, joy):

        self.speed_dir_spin = joy.axes[4]

        if self.upper_switch == 1:  # upper limit switch pressed

            if joy.axes[1] <= 0:  # allowing you to move down, but not up
                self.speed_dir = joy.axes[1]

            else:
                self.speed_dir = 0  # if still trying to move up do nothing

        elif self.lower_switch == 1:  # lower limit switch pressed

            if joy.axes[1] >= 0:  # allowing you to move up, but not down
                self.speed_dir = joy.axes[1]

            else:
                self.speed_dir = 0  # if still trying to move down do nothing

        else:
            self.speed_dir = joy.axes[1]
            # if neither sensor is pressed,
            # movement in either direction is allowed

        if joy.buttons[2] == 1:  # A button
            self.position = 1  # open

        elif joy.buttons[1] == 1:  # B button
            self.position = 2  # close

        self.send_info()

    def send_info(self):

        canFrame = self.send_flap()
        canFrame2 = self.send_spin()
        canFrame3 = self.send_limit()

        self.pub.publish(canFrame)
        self.pub.publish(canFrame2)
        self.pub.publish(canFrame3)

    def send_flap(self):

        frame = Frame()
        frame.id = self.ID_flap
        frame.is_rtr = False
        frame.is_extended = False
        frame.is_error = False
        frame.dlc = 8
        data = bytearray(struct.pack('i', self.position))
        frame.data = str(data)
        return frame

    def send_spin(self):

        frame = Frame()
        frame.id = self.ID_spin
        frame.is_rtr = False
        frame.is_extended = False
        frame.is_error = False
        frame.dlc = 8
        data = bytearray(struct.pack('f', self.speed_dir_spin))
        frame.data = str(data)
        return frame

    def send_limit(self):

        frame = Frame()
        frame.id = self.ID_limit
        frame.is_rtr = False
        frame.is_extended = False
        frame.is_error = False
        frame.dlc = 8
        data = bytearray(struct.pack('f', self.speed_dir))
        frame.data = str(data)
        return frame


if __name__ == '__main__':
    controller = ScienceController()
    rospy.spin()
