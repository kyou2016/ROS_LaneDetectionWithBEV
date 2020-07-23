#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan, Imu, PointCloud
from vesc_msgs.msg import VescStateStamped
from laser_geometry import LaserProjection
from math import cos, sin, pi, sqrt, pow
from geometry_msgs.msg import Point32, PoseStamped
from nav_msgs.msg import Path, Odometry

class make_path:
    
    def __init__(self):
        rospy.init_node('make_path', anonymous=True)

        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.path_pub = rospy.Publisher('/path', Path, queue_size=1)
        self.is_odom = False
        self.path_msg = Path()
        self.path_msg.header.frame_id = '/odom'
        self.prev_x = 0
        self.prev_y = 0
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('example')
        full_path = pkg_path + '/path' + '/path.txt'
        self.f = open(full_path, 'w')

        while not rospy.is_shutdown():
            rospy.spin()
        
        self.f.close()

    def odom_callback(self, msg):
        waypint_pose = PoseStamped()
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.is_odom == True:
            distance = sqrt(pow(x - self.prev_x, 2) + pow(y - self.prev_y, 2))
            if distance > 0.1:
                waypint_pose.pose.position.x = x
                waypint_pose.pose.position.y = y
                waypint_pose.pose.orientation.w = 1
                self.path_msg.poses.append(waypint_pose)
                self.path_pub.publish(self.path_msg)
                data = '{0}\t{1}\n'.format(x,y)
                self.f.write(data)
                self.prev_x = x
                self.prev_y = y
                print(x, y)
        else:
            self.is_odom = True
            self.prev_x = x
            self.prev_y = y

if __name__ == '__main__':
    try:
        test_track = make_path()
    except rospy.ROSInterruptException:
        pass