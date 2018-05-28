#!/usr/bin/env python
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt8
from can_msgs.msg import Frame
import rospy
import struct


class Science:
    """
A to open flap, B to close flap. One joy stick to control the spin and
the other joy stick to control the vertical movement. X to read temp sensor
and Y to read EC sensor. RT to move rack and pinion system up for
temp probe motor. RB to move rack and pinion system down for temp
probe motor.
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
        self.temperature_switch = 0
        self.speed_dir = 0
        self.speed_dir_spin = 0
        self.position = 999
        self.which_sensor = 999
        self.temp_sensor = 0
        self.ID_limit = 420
        self.ID_spin = 421
        self.ID_temp_sensor = 422
        self.ID_flap = 423
        self.ID_read_sensor = 510

    def callback_switches(self, switches):

        self.upper_switch = (switches.data) & 0x1
        # check to see if the upper switch is being pressed

        self.lower_switch = (switches.data >> 1) & 0x1
        # check to see if the lower switch is being pressed

        self.temperature_switch = (switches.data >> 2) & 0x1
        # check to see if limit switch for temp probe is being pressed

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

        shifter = rospy.get_param('budge')  # motor increment value

        if self.temperature_switch == 1:

            if joy.buttons[5] == 1:  # RB is pushed trying to go down
                self.temp_sensor -= shifter

            elif joy.buttons[7] == 1:  # RT button to move up
				self.temp_sensor = 0  # do not move

        else:
            if joy.buttons[5] == 1:
                self.temp_sensor -= shifter

            if joy.buttons[7] == 1:
                self.temp_sensor += shifter

        self.temp_sensor = min(max(self.temp_sensor, -1), 1)

        if joy.buttons[2] == 1:  # A button
            self.position = 1  # open

        elif joy.buttons[1] == 1:  # B button
            self.position = 0  # close

        if joy.buttons[3] == 1:  # X Button
            self.which_sensor = 1  # EC sensor

        elif joy.buttons[0] == 1:  # Y Button
            self.which_sensor = 0  # Temp+Humidity sensor

        if joy.buttons[2] == 1 or joy.buttons[1] == 1:
            self.send_info_1()

        if joy.axes[4] != 0:
            self.send_info_2()

        if joy.axes[1] != 0:
            self.send_info_3()

        if joy.buttons[3] == 1 or joy.buttons[0] == 1:
            self.send_info_4()

        if joy.buttons[7] == 1 or joy.buttons[5] == 1:
            self.send_info_5()

    def send_info_1(self):

        canFrame = self.send_flap()
        self.pub.publish(canFrame)

    def send_info_2(self):

        canFrame2 = self.send_spin()
        self.pub.publish(canFrame2)

    def send_info_3(self):

        canFrame3 = self.send_limit()
        self.pub.publish(canFrame3)

    def send_info_4(self):

        canFrame4 = self.send_read_sensor()
        self.pub.publish(canFrame4)

    def send_info_5(self):

        canFrame5 = self.send_temp_sensor()
        self.pub.publish(canFrame5)

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

    def send_read_sensor(self):

        frame = Frame()
        frame.id = self.ID_read_sensor
        frame.is_rtr = False
        frame.is_extended = False
        frame.is_error = False
        frame.dlc = 8
        data = bytearray(struct.pack('i', self.which_sensor))
        frame.data = str(data)
        return frame

    def send_temp_sensor(self):

        frame = Frame()
        frame.id = self.ID_temp_sensor
        frame.is_rtr = False
        frame.is_extended = False
        frame.is_error = False
        frame.dlc = 8
        data = bytearray(struct.pack('f', self.temp_sensor))
        frame.data = str(data)
        return frame


if __name__ == '__main__':
    controller = Science()
    rospy.spin()
