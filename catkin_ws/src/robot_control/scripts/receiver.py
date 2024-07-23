#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(msg):
    rospy.loginfo("Received command: %s", msg.data)

def receiver():
    rospy.init_node('receiver_node')
    rospy.Subscriber('command_topic', String, callback)
    rospy.spin()

if __name__ == '__main__':
    receiver()
