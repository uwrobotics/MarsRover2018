#!/usr/bin/env python
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt8
from can_msgs.msg import Frame
import rospy
import struct


class Science:
    """
A to open flap, B to close aflap. One joy stick to control the spin and
the other joy stick to control the vertical movement. X to read temp sensor
and Y to read EC sensor. RT to move rack and pinion system up for
temp probe motor. RB to move rack and pinion system down for temp
probe motor.
Author: Zameer Bharwani
"""

    def __init__(self):

        self.init_Auger = rospy.init_node("ScienceController", anonymous=True)
        self.sub = rospy.Subscriber("/science_joy", Joy, self.callback_xbox)
        self.pub = rospy.Publisher("/CAN_transmitter", Frame, queue_size=20)
        self.upper_switch = 0
        self.lower_switch = 0
        self.temperature_switch = 0
        self.speed_dir = 0
        self.speed_dir_spin = 0
        self.position = 0
        self.which_sensor = 0
        self.temp_sensor = 0
        self.ID_auger = 420
        self.ID_spin = 421
        self.ID_temp_sensor = 422
        self.ID_flap = 423
        self.ID_read_sensor = 510

    def callback_xbox(self, joy):

        if joy.buttons[4] == 1:

            self.speed_dir_spin = 1  # spin forward

        elif joy.buttons[3] == 1:
            self.speed_dir_spin = -1  # spin backward

        else:
            self.speed_dir_spin = 0

        self.speed_dir = joy.axes[1]

        if joy.buttons[7] == 1:
            self.temp_sensor = -1

        elif joy.buttons[8] == 1:
            self.temp_sensor = 1

        else:
            self.temp_sensor = 0

        if joy.buttons[10] == 1:  # A button
            self.position = 1  # open

        elif joy.buttons[9] == 1:  # B button
            self.position = 0  # close

        if joy.buttons[5] == 1:  # X Button
            self.which_sensor = 1  # EC sensor

        elif joy.buttons[6] == 1:  # Y Button
            self.which_sensor = 0  # Temp+Humidity sensor

        if joy.buttons[10] == 1 or joy.buttons[9] == 1:
            self.send_info_flap()

        if joy.buttons[5] == 1 or joy.buttons[6] == 1:
            self.send_info_sensor_select()

        self.send_info_drill()

        self.send_info_auger()

        self.send_info_temp_sensor()

    def send_info_flap(self):

        canFrame_flap = self.send_flap()
        self.pub.publish(canFrame_flap)

    def send_info_drill(self):

        canFrame_spin = self.send_spin()
        self.pub.publish(canFrame_spin)

    def send_info_auger(self):

        canFrame_auger = self.send_auger()
        self.pub.publish(canFrame_auger)

    def send_info_sensor_select(self):

        canFrame_sensor = self.send_read_sensor()
        self.pub.publish(canFrame_sensor)

    def send_info_temp_sensor(self):

        canFrame_temp_sensor = self.send_temp_sensor()
        self.pub.publish(canFrame_temp_sensor)

    def send_flap(self):

        frame = Frame()
        frame.id = self.ID_flap
        frame.is_rtr = False
        frame.is_extended = False
        frame.is_error = False
        frame.dlc = 4
        data = bytearray(struct.pack('i', self.position))
        frame.data = str(data)
        return frame

    def send_spin(self):

        frame = Frame()
        frame.id = self.ID_spin
        frame.is_rtr = False
        frame.is_extended = False
        frame.is_error = False
        frame.dlc = 4
        data = bytearray(struct.pack('i', self.speed_dir_spin))
        frame.data = str(data)
        return frame

    def send_auger(self):

        frame = Frame()
        frame.id = self.ID_auger
        frame.is_rtr = False
        frame.is_extended = False
        frame.is_error = False
        frame.dlc = 4
        data = bytearray(struct.pack('f', self.speed_dir))
        frame.data = str(data)
        return frame

    def send_read_sensor(self):

        frame = Frame()
        frame.id = self.ID_read_sensor
        frame.is_rtr = False
        frame.is_extended = False
        frame.is_error = False
        frame.dlc = 4
        data = bytearray(struct.pack('i', self.which_sensor))
        frame.data = str(data)
        return frame

    def send_temp_sensor(self):

        frame = Frame()
        frame.id = self.ID_temp_sensor
        frame.is_rtr = False
        frame.is_extended = False
        frame.is_error = False
        frame.dlc = 4
        data = bytearray(struct.pack('i', self.temp_sensor))
        frame.data = str(data)
        return frame


if __name__ == '__main__':
    controller = Science()
    rospy.spin()
