#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan, PointCloud
from vesc_msgs.msg import VescStateStamped
from laser_geometry import LaserProjection
from math import cos, sin, pi
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry


import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler



class simple_kinematics:
    def __init__(self):
        rospy.init_node('simple_kinematics', anonymous=True)

        rospy.Subscriber('/sensors/core', VescStateStamped, self.status_callback)
        rospy.Subscriber('/sensors/servo_position_command', Float64, self.servo_command_callback)

        self.is_speed = False
        self.is_servo = False
        self.servo_msg = Float64()

        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = '/odom'

        self.rpm_gain = 4614
        self.steering_angle_to_servo_gain = -1.2135
        self.sterring_angle_to_servo_offset = 0.5304
        self.theta = 0
        self.L = 0.5

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():

            if self.is_servo == True and self.is_speed == True:
                print(self.speed, self.servo_angle_rad*180/pi)

                x_dot = self.speed*cos(self.theta + self.servo_angle_rad)/20
                y_dot = self.speed*sin(self.theta + self.servo_angle_rad)/20
                theta_dot = self.speed*sin(self.servo_angle_rad)/self.L/20

                self.odom_msg.pose.pose.position.x = self.odom_msg.pose.pose.position.x + x_dot
                self.odom_msg.pose.pose.position.y = self.odom_msg.pose.pose.position.y + y_dot
                self.theta = self.theta + theta_dot
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
                    "base_link", "odom")
            rate.sleep()

    def status_callback(self, msg):
        self.is_speed = True
        rpm = msg.state.speed
        self.speed = rpm/self.rpm_gain
    
    def servo_command_callback(self, msg):
        self.is_servo = True
        servo_value = msg.data
        self.servo_angle_rad = (servo_value - self.sterring_angle_to_servo_offset)/self.steering_angle_to_servo_gain


if __name__ == '__main__':
    try:
        test_track = simple_kinematics()
    except rospy.ROSInterruptException:
        pass