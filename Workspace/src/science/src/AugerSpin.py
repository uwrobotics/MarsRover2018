#!/usr/bin/env python
from sensor_msgs.msg import Joy
from can_msgs.msg import Frame
import rospy
import struct

class Auger_Spin:

	"""Auger Spinning forward and backward control. Joystick up = soil samples will be dug up. """
	def __init__(self):

		self.init_Auger_spin=rospy.init_node("AugerSpinDirection", anonymous=True)
		self.sub=rospy.Subscriber("/joy", Joy, self.callback)
		self.pub=rospy.Publisher("/auger_spin_dir", Frame, queue_size=20)
		self.speed_dir=0
		self.ID=600


	def callback(self,joy):

		self.speed_dir=joy.axes[4]

		self.send_speed_dir()

	def send_speed_dir(self):

		canFrame = self.Spin_To_Frame()
		self.pub.publish(canFrame)

	
	def Spin_To_Frame(self):

		frame = Frame()
		frame.id = self.ID
		frame.is_rtr = False
		frame.is_extended = False
		frame.is_error = False
		frame.dlc = 8
		data = bytearray(struct.pack('f', self.speed_dir))
		frame.data = str(data)
		return frame


if __name__ == '__main__':
	controller = Auger_Spin()
	rospy.spin()

	


	
	










