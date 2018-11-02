#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32

def callback(data):
    pub = rospy.Publisher('/assignment1_publisher_subscriber', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    hello_str = "Current Yaw %s" % data
    pub.publish(hello_str)
    rospy.loginfo(data)


def listener():
    print("Starting listener")
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/yaw", Float32, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
