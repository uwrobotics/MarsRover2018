#!/usr/bin/env python
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt8
from can_msgs.msg import Frame
import rospy
import struct

class Auger:

	"""The limit switches tell us when we've moved the platform too high or too low - they should turn off the lead screw motor immediately. 
	The lead screw motor controls the auger. Use left joystick."""
	
	def __init__(self):

		self.init_Auger=rospy.init_node("Auger", anonymous=True)
		self.sub_switches=rospy.Subscriber("/science_limit_switches",UInt8,self.callback_switches) 
		self.sub_xbox=rospy.Subscriber("/joy", Joy, self.callback_xbox)
		self.pub_auger=rospy.Publisher("/auger", Frame, queue_size=20)
		self.upper_switch=0
		self.lower_switch=0
		self.speed_dir=0  # speed_dir = speed, direction. +1= forward at full speed, -1 = backwards at full speed and all values in between
		self.ID=600
	

	def callback_switches(self,switches):

		if (switches.data)&0x1==1: #check to see if the upper switch is being pressed
			self.upper_switch=1

		else:
			self.upper_switch=0


		if (switches.data>>1)&0x1==1: #check to see if the lower switch is being pressed
			self.lower_switch=1

		else:
			self.lower_switch=0
		
	def callback_xbox(self,joy):

		if self.upper_switch==1: 		#upper limit switch pressed
			if joy.axes[1]<=0: 		#allowing you to move down, but not up
				self.speed_dir=joy.axes[1]
			else:
				self.speed_dir=0 #if still trying to move up do nothing 
		
		elif self.lower_switch==1: 		#lower limit switch pressed
			if joy.axes[1]>=0:		#allowing you to move up, but not down
				self.speed_dir=joy.axes[1]
			else:
				self.speed_dir=0 			#if still trying to move down do nothing 

		else:
			self.speed_dir=joy.axes[1] 		#if neither sensor is pressed, movement in either direction is allowed

		self.send_speed_dir()

	def send_speed_dir(self):

		canFrame = self.speed_dir_To_Frame()
		self.pub_auger.publish(canFrame)

	
	def speed_dir_To_Frame(self):

		frame = Frame()
		frame.id = self.ID
		frame.is_rtr = False
		frame.is_extended = False
		frame.is_error = False
		frame.dlc = 8
		data = bytearray(struct.pack('i', self.speed_dir))
		frame.data = str(data)
		return frame


if __name__ == '__main__':
	controller = Auger()
	rospy.spin()

	


	
	










