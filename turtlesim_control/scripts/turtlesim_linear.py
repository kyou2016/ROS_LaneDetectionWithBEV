#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time


name_node = 'turtlesim_linear'

class LINEARController:
    def __init__(self, ref=3, spd=1):
        self.ref = ref
        self.spd = spd
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

        self.pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.sub = rospy.Subscriber(self.pose_topic, Pose, self.pose_callback)

    def pose_callback(self, msgs):
        self.x = msgs.x
        self.y = msgs.y
        self.yaw = msgs.theta

    def pub_cmd_msgs(self):
        self.dist_move = math.sqrt((self.x - self.x0)**2 + (self.y - self.y0)**2)
        if self.dist_move <= self.ref:
            rospy.loginfo(str(self.dist_move))
            self.cmd_vel_msgs.linear.x = abs(self.spd)
        else:
            self.cmd_vel_msgs.linear.x = 0
        self.pub.publish(self.cmd_vel_msgs)

    def init_pose(self):
        self.x0 = self.x
        self.y0 = self.y
        self.yaw0 = self.yaw

if __name__ == '__main__':
    rospy.init_node(name_node)
    linear_ctrl = LINEARController()
    time.sleep(2)
    rate = rospy.Rate(10)
    linear_ctrl.init_pose()
    while not rospy.is_shutdown():
        linear_ctrl.pub_cmd_msgs()
        rate.sleep()