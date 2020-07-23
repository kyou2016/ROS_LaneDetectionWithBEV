#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan, PointCloud
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from geometry_msgs.msg import Point32
from math import cos, sin, pi

class simple_controller3:
    def __init__(self):
        rospy.init_node('simple_controller3', anonymous=True)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.motor_msg = Float64()
        self.servo_msg = Float64()

        self.motor_pub = rospy.Publisher('commands/motor/speed', Float64, queue_size=1)
        self.servo_pub = rospy.Publisher('commands/servo/position', Float64, queue_size=1)
        self.pcd_pub = rospy.Publisher('laser2pcd', PointCloud, queue_size=1)

        self.motor_msg.data = 3000.0
        self.servo_msg.data = 0.5304
        self.motor_pub.publish(self.motor_msg)
        self.servo_pub.publish(self.servo_msg)

        while not rospy.is_shutdown():
            rospy.spin()

    def laser_callback(self, msg):
        pcd = PointCloud()
        pcd.header.frame_id = msg.header.frame_id
        angle = 0

        for r in msg.ranges:
            tmp_point=Point32()
            tmp_point.x = r*cos(angle)
            tmp_point.y = r*sin(angle)
            angle = angle+(1.0/180*pi)
            if r<12:
                pcd.points.append(tmp_point)

        count = 0
        countL = 0
        countR = 0
        for point in pcd.points:
            if point.x > 0 and point.x < 1.85 and point.y > -1.0 and point.y < 1.0:
                count += 1
            if point.x > 0 and point.x < 1 and point.y > 0:
                countL += 1
            if point.x > 0 and point.x < 1 and point.y < 0:
                countR += 1

        if count > 20:
            self.motor_msg.data=2000.0
            if countL > countR:
                print("Right")
                self.servo_msg.data=0.85
            elif countR > countL:
                print("Left")
                self.servo_msg.data=0.15
        else:
            print("Go")
            self.motor_msg.data=3000.0
            self.servo_msg.data=0.5304

        print(count, countL, countR)
        self.motor_pub.publish(self.motor_msg)
        self.servo_pub.publish(self.servo_msg)
        self.pcd_pub.publish(pcd)        

if __name__ == '__main__':
    try:
        test_track = simple_controller3()
    except rospy.ROSInterruptException:
        pass