#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time

name_node = 'turtlesim_position_control'

class POSITIONControl:
    def __init__(self, refx=3, refy=10, K_linear=0.5, K_rotate=2):
        self.refx = refx
        self.refy = refy
        self.K_linear = K_linear
        self.K_rotate = K_rotate
        self.cmd_vel_msgs = Twist()
        self.cmd_vel_topic = '/turtle1/cmd_vel'
        self.pose_topic = '/turtle1/pose'

        self.x = None
        self.y = None
        self.yaw = None
        
        self.sub = rospy.Subscriber(self.pose_topic, Pose, self.pose_callback)
        self.pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)

    def pose_callback(self, msgs):
        self.x = msgs.x
        self.y = msgs.y
        self.yaw = msgs.theta

    def pub_cmd_vel_msgs(self):
        err_dist = math.sqrt((self.refx - self.x)**2 + (self.refy - self.y)**2)
        yaw_dist = math.atan2(self.refy - self.y, self.refx - self.x)
        err_yaw = yaw_dist - self.yaw
        if err_dist > 0.0:         
            self.cmd_vel_msgs.linear.x = err_dist*self.K_linear
            self.cmd_vel_msgs.angular.z = err_yaw*self.K_rotate   
        else:
            self.cmd_vel_msgs.linear.x = 0
            self.cmd_vel_msgs.angular.z = 0
        self.pub.publish(self.cmd_vel_msgs)
        
if __name__ == '__main__':
    rospy.init_node(name_node)
    position_ctrl = POSITIONControl(refx=3, refy=8)
    time.sleep(2)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        position_ctrl.pub_cmd_vel_msgs()
        rate.sleep()
