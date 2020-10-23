#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan, PointCloud, Imu
from vesc_msgs.msg import VescStateStamped
from laser_geometry import LaserProjection
from math import cos, sin, pi
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry


import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class imu_odom:
    def __init__(self):
        rospy.init_node('imu_odom', anonymous=True)

        rospy.Subscriber('/vesc/sensors/core', VescStateStamped, self.status_callback)
	rospy.Subscriber('/vesc/odom',Odometry,self.odom_callback)

        self.is_speed = False
	self.is_vesc_odom=False

        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = '/odom'

        self.rpm_gain = 4614
        self.theta = 0

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            if self.is_vesc_odom == True and self.is_speed == True:
                print(self.speed, self.theta*180/pi)
                
               

                self.odom_pub.publish(self.odom_msg)
		quaternion =quaternion_from_euler(0, 0, self.theta)
                br = tf.TransformBroadcaster()
                br.sendTransform((self.odom_msg.pose.pose.position.x, 					self.odom_msg.pose.pose.position.y, 					self.odom_msg.pose.pose.position.z),
                     		quaternion,   
                     		rospy.Time.now(),
                     		"base_link",
                     		"map")

            rate.sleep()

    def status_callback(self, msg):
        self.is_speed = True
        rpm = msg.state.speed
        self.speed = rpm/self.rpm_gain
    def odom_callback(self,msg):
        self.odom_msg=msg
	vesc_quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 					msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
	if self.is_vesc_odom == False:
            self.is_vesc_odom = True
            _, _, self.theta_offset = euler_from_quaternion(vesc_quaternion)
        else:
            _, _, raw_theta = euler_from_quaternion(vesc_quaternion)
            self.theta = raw_theta - self.theta_offset
  
        

if __name__ == '__main__':
    try:
        test_track = imu_odom()
    except rospy.ROSInterruptException:
        pass
