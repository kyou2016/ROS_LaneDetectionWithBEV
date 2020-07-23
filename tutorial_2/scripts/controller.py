#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32

name_node = 'controller'

class PController:
    def __init__(self, ref=10, T=0.1, K=1, Ki=100):
        self.T = T
        self.ref = ref
        self.K = K
        self.Ki = Ki
        self.ctrl_msgs = Float32()
        self.ctrl_topic = '/pid_ctrl'
        self.measure_topic = '/measure_plant'

        self.err_i = 0.0
        self.y = 0.0

        self.sub = rospy.Subscriber(self.measure_topic,Float32,self.measure_callback)
        self.pub = rospy.Publisher(self.ctrl_topic,Float32,queue_size=10)

    def measure_callback(self, msgs):
        self.y = msgs.data

    def pub_ctrl_msgs(self):
        err = self.ref - self.y
        self.err_i += self.T*err
        u = self.K * err + self.Ki * self.err_i
        self.ctrl_msgs.data = u
        self.pub.publish(self.ctrl_msgs)

if __name__ == '__main__':
    rospy.init_node(name_node)
    controller = PController()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        controller.pub_ctrl_msgs()
        rate.sleep()