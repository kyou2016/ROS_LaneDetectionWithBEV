#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time

name_node = 'turtlesim_rotate'

class ROTATEController:
    def __init__(self, ref=math.pi/2, wspd=1):
        self.ref = ref
        self.wspd = wspd
        self.dist_move = 0
        self.cmd_vel_msgs = Twist()
        self.cmd_vel_topic = '/turtle1/cmd_vel'
        self.pose_topic = '/turtle1/pose'

        self.x = None
        self.y = None
        self.yaw = None

        self.x0 = None
        self.y0 = None
        self.yaw0 = None

        self.sub = rospy.Subscriber(self.pose_topic, Pose, self.pose_callback)
        self.pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
    
    def pose_callback(self, msgs):
        self.x = msgs.x
        self.y = msgs.y
        self.yaw = msgs.theta

    def pub_cmd_msgs(self):
        self.dist_move = self.yaw - self.yaw0
        if self.dist_move <= self.ref:
            rospy.loginfo(str(self.dist_move))
            self.cmd_vel_msgs.angular.z = abs(self.wspd)
        else:
            self.cmd_vel_msgs.angular.z = 0
        self.pub.publish(self.cmd_vel_msgs)
    
    def init_pose(self):
        self.x0 = self.x
        self.y0 = self.y
        self.yaw0 = self.yaw

if __name__ == '__main__':
    rospy.init_node(name_node)
    rotate_ctrl = ROTATEController()
    time.sleep(2)
    rate = rospy.Rate(10)
    rotate_ctrl.init_pose()
    while not rospy.is_shutdown():
        rotate_ctrl.pub_cmd_msgs()
        rate.sleep()