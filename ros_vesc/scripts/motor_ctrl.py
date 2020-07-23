#!/usr/bin/env python
#from __future__ import print_function

import roslib
import sys
import rospy
from std_msgs.msg import Float64



class motor_control:
	
	def __init__(self):
	 print('aaaaa')
	 self.rate = rospy.Rate(10)
	 self.timer_to_sending_data=0
	 
	 self.speed = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
	 self.position = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)

	 while not rospy.is_shutdown(): 
	  self.speed_value=1200
	  self.position_value=0.5
	  self.speed.publish(self.speed_value)
	  self.position.publish(self.position_value)
	  self.rate.sleep()


def main(args):
	rospy.init_node('motor_control',anonymous=True)

	motor_control()
	rospy.spin()

if __name__=='__main__':
	
	main(sys.argv)
	

