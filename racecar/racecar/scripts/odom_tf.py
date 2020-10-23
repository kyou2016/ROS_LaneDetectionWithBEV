#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan,PointCloud,Imu
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from laser_geometry import LaserProjection
from math import cos,sin,pi
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry

import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler

class odom_transfrom:

    def __init__(self):
        rospy.init_node('odom_transfrom',anonymous=True)

        
        rospy.Subscriber('/vesc/odom',Odometry,self.odom_callback)

        self.is_vesc_odom=False

        self.odom_pub = rospy.Publisher('/odom',Odometry,queue_size=1)
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id ='map'

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():

            if self.is_vesc_odom == True:
                # print(self.odom_msg.pose.pose.position)
                
		br = tf.TransformBroadcaster()
		br.sendTransform((self.odom_msg.pose.pose.position.x, self.odom_msg.pose.pose.position.y, self.odom_msg.pose.pose.position.z), 
				self.quaternion,
				rospy.Time.now(),
				"base_link",
				"map")
		self.odom_pub.publish(self.odom_msg)
                rate.sleep()

    def odom_callback(self,msg):
        self.is_vesc_odom=True
        self.odom_msg=msg
	self.odom_msg.header.frame_id ='map'
	self.quaternion = [self.odom_msg.pose.pose.orientation.x, self.odom_msg.pose.pose.orientation.y, self.odom_msg.pose.pose.orientation.z, self.odom_msg.pose.pose.orientation.w]

   
if __name__ =='__main__':
    try:
        test_track =odom_transfrom()
    except rospy.ROSInterruptException:
        pass
