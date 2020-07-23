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

        rospy.Subscriber('/sensors/core', VescStateStamped, self.status_callback)
        rospy.Subscriber('/imu', Imu, self.imu_callback)

        self.is_speed = False
        self.is_imu = False

        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = '/odom'

        self.rpm_gain = 4614
        self.theta = 0

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            if self.is_imu == True and self.is_speed == True:
                print(self.speed, self.theta*180/pi)
                
                ## theta의 변화량은 각도 -> 각속도(rad/s)를 시간으로 적분하면 각도를 알 수 있음, 현재 반복되는 시간은 1/20초 -> 0.05초
                ## theta에 변화량을 더해줘서 값을 누적한다.
                theta_dot = self.imu_angular*0.05
                self.theta += theta_dot
                x_dot = self.speed * cos(self.theta)/20
                y_dot = self.speed * sin(self.theta)/20

                self.odom_msg.pose.pose.position.x += x_dot
                self.odom_msg.pose.pose.position.y += y_dot

                quaternion = quaternion_from_euler(0, 0, self.theta)
                self.odom_msg.pose.pose.orientation.x = quaternion[0]
                self.odom_msg.pose.pose.orientation.y = quaternion[1]
                self.odom_msg.pose.pose.orientation.z = quaternion[2]
                self.odom_msg.pose.pose.orientation.w = quaternion[3]

                self.odom_pub.publish(self.odom_msg)
                br = tf.TransformBroadcaster()
                br.sendTransform((self.odom_msg.pose.pose.position.x, self.odom_msg.pose.pose.position.y, self.odom_msg.pose.pose.position.z),
                     quaternion,   
                     rospy.Time.now(),
                     "base_link",
                     "odom")

            rate.sleep()

    def status_callback(self, msg):
        self.is_speed = True
        rpm = msg.state.speed
        self.speed = rpm/self.rpm_gain
    
    def imu_callback(self, msg):
        # imu_quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        # if self.is_imu == False:
        #     self.is_imu = True
        #     _, _, self.theta_offset = euler_from_quaternion(imu_quaternion)
        #     self.theta = self.theta + self.imu_angular
        # # else:
        #     _, _, raw_theta = euler_from_quaternion(imu_angular)
        #     self.theta = raw_theta - self.theta_offset
        self.imu_angular = msg.angular_velocity.z ## 각속도의 단위는 rad/s -> 1초당 각도(rad)
        if self.is_imu == False:
            self.is_imu = True
        

if __name__ == '__main__':
    try:
        test_track = imu_odom()
    except rospy.ROSInterruptException:
        pass