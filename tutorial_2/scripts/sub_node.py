#!/usr/bin/env python
import rospy
from tutorial_2.msg import MyMsgs

NAME_TOPIC = '/msgs_talk'
NAME_NODE = 'sub_node'

def callback(msgs):
    rospy.loginfo(msgs.x[0] + 2*msgs.y[0])

if __name__ == '__main__':
    rospy.init_node(NAME_NODE, anonymous=True)
    sub = rospy.Subscriber(NAME_TOPIC, MyMsgs, callback)
    rospy.spin()