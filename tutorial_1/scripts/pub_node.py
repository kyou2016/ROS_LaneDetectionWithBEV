import rospy
from std_msgs.msg import String

NAME_TOPIC = '/msgs_talk'
NAME_NODE = 'pub_node'

if __name__ == '__main__':
    rospy.init_node(NAME_NODE, anonymous=True)
    pub = rospy.Publisher(NAME_TOPIC, String, queue_size=10)
    rate = rospy.Rate(10)
    msgs_pub = String()
    while not rospy.is_shutdown():
        msgs_pub.data = "Hello ROS"
        pub.publish(msgs_pub)
        rate.sleep()