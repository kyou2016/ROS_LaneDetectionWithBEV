#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import Float64, String

class joycom:
    def __init__(self):
        self.speed_sub = rospy.Subscriber('/vesc/commands/motor/speed',Float64,self.speed_callback)
        self.steer_sub = rospy.Subscriber('/vesc/commands/servo/position',Float64,self.steer_callback)
        
        self.speed_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
        self.steer_pub = rospy.Publisher('/commands/servo/position',Float64, queue_size=1)

        self.speed = Float64()
        self.steer = Float64()

    def steer_callback(self,msg):
        self.steer = msg
        print(msg)

    def speed_callback(self,msg):
	    self.speed = msg
	    print(msg)

    def rospub(self):       
        self.speed_pub.publish(self.speed)
        self.steer_pub.publish(self.steer)

if __name__ == '__main__':
    rospy.init_node('joycom', anonymous=True)

    wecar = joycom()

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        wecar.rospub()
        rate.sleep()
