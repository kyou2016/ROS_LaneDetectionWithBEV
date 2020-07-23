#!/usr/bin/env python

import rospy
import sys
import roslib
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan

namenode = "demo_control"

class demo_control:
	def __init__(self):
		self.timer_to_sending_data = 0
        self.direction = "front"

        self.speed = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)
        self.position = rospy.Publisher("/commands/servo/position", Float64, queue_size=1)
		
        self.speed_value = 1200
        self.position_value = 0.5

        self.scan_data = rospy.Subscriber("scan", LaserScan, self.publish_Control_Data)
		
        self.position.publish(self.position_value)

    def publish_Control_Data(self, data):
        for i in range(0, 360):
            lidar_dis = ranges[i]
            if (i > 100) and (i < 180) and (lidar_dis < 0.5):
                self.direction = "left"

            elif (i > 180) and (i < 220) and (lidar_dis < 0.5):
                self.direction = "right"

            elif (i == 180) and (lidar_dis < 0.5):
                self.direction = "front"

        if self.direction == "left":
            print("Left Wall!! value", lidar_dis)
            self.speed_value = 1200
            if self.position_value > 0.0:
                self.position_value -= 0.025
                
        elif self.direction == "right":
            print("Right Wall!! value", lidar_dis)
            self.speed_value = 1200
            if self.position_value < 1.0:
                self.position_value += 0.025

        elif self.direction == "front":
            print("Front Wall!! value", lidar_dis)
            self.speed_value = -1200
            self.position_value = 0.5

        self.speed.publish(self.speed_value)
        self.position.publish(self.position_value)


if __name__ == '__main__':
	rospy.init_node(namenode)
	demo_ctrl = demo_control()
	rate = rospy.Rate(1)
	rospy.spin()