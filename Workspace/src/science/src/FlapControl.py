#!/usr/bin/env python
from sensor_msgs.msg import Joy
from can_msgs.msg import Frame
import rospy
import struct

class Flap:

	"""Open and close flap. Push A to open, B to close """

	def __init__(self):

		self.init=rospy.init_node("Flap", anonymous=True)
		self.sub=rospy.Subscriber("/joy",Joy, self.callback)
		self.pub=rospy.Publisher("/flap", Frame, queue_size=20)
		self.position=0
		self.ID = 600 # CAN id



	def callback(self, joy):

		if joy.buttons[2]==1: #A button 

			self.position=1 #open

		elif joy.buttons[1]==1: #B button

			self.position=2 #close 

		self.send()

	def send(self):
		canFrame = self.Position_To_Frame()
		self.pub.publish(canFrame)


	def Position_To_Frame(self):

		frame = Frame()
		frame.id = self.ID
		frame.is_rtr = False
		frame.is_extended = False
		frame.is_error = False
		frame.dlc = 8
		data = bytearray(struct.pack('i', self.position))
		frame.data = str(data)
		return frame


if __name__ == '__main__':
	controller = Flap()
	rospy.spin()



			







