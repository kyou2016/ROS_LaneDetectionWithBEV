#!/usr/bin/env python

import rospy
import sys
import roslib
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan

namenode = "motor_control"

class motor_control:

	def __init__(self):
		self.timer_to_sending_data = 0

		self.speed = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)
		# self.position = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
		
		self.scan_data = rospy.Subscriber("scan", LaserScan, self.publish_Control)
		
	def publish_Control(self, data):
		for i in range(170, 190):
			lidar_dis = data.ranges[180]		
			if (lidar_dis > 0.0) & (lidar_dis <= 0.5):
				print("Go, value", lidar_dis)
				self.speed_value = 1100
				# self.position_value = 0.5
				self.speed.publish(self.speed_value)
				# self.position.publish(self.position_value)
			else:
				print("Stop, value", lidar_dis)
				self.speed_value = 0
				self.speed.publish(self.speed_value)
			print('')
		


if __name__ == '__main__':
	rospy.init_node(namenode)
	motor_ctrl = motor_control()
	rate = rospy.Rate(1)
	rospy.spin()