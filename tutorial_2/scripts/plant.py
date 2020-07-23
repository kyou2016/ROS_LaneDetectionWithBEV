#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32

name_node = 'plant'


class PLANT:
    def __init__(self, T=0.1, a=0.2):
        self.T = T
        self.a = a
        self.measure_msgs = Float32()
        self.ctrl_topic = '/pid_ctrl'
        self.measure_topic = '/measure_plant'

        self.x = 0.0
        self.u = 0.0

        self.sub = rospy.Subscriber(self.ctrl_topic, Float32, self.ctrl_callback)
        self.pub = rospy.Publisher(self.measure_topic, Float32, queue_size=10)

    def ctrl_callback(self, msgs):
        self.u = msgs.data

    def pub_measure_msgs(self):
        self.x = self.x - self.T*self.a*self.x + self.T*self.u
        self.measure_msgs.data = self.x

        self.pub.publish(self.measure_msgs)

if __name__ == '__main__':
    rospy.init_node(name_node)
    plant = PLANT()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        plant.pub_measure_msgs()
        rate.sleep()