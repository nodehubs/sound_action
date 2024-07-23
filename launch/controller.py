#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def callback(msg, pub):
    command = msg.data
    twist = Twist()
    
    move_duration = 2  # 持续移动时间 (秒)
    turn_duration = 6.5  # 转弯时间 (秒)
    stop_duration = 6.0  # 停留的持续时间（秒）
    rate = rospy.Rate(10)  # 10 Hz

    if command == "取药":
        rospy.loginfo("Moving forward to pick up medicine")
        
        # 向前移动
        start_time = rospy.get_time()
        while rospy.get_time() - start_time < move_duration and not rospy.is_shutdown():
            twist.linear.x = 0.5
            twist.angular.z = 0.0
            pub.publish(twist)
            rate.sleep()
        # 停留
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        rospy.sleep(stop_duration)

        # 转弯
        start_time = rospy.get_time()
        while rospy.get_time() - start_time < turn_duration and not rospy.is_shutdown():
            twist.linear.x = 0.0
            twist.angular.z = 0.5
            pub.publish(twist)
            rate.sleep()

        # 停留
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        rospy.sleep(stop_duration)
        
        # 向前移动
        start_time = rospy.get_time()
        while rospy.get_time() - start_time < move_duration and not rospy.is_shutdown():
            twist.linear.x = 0.5
            twist.angular.z = 0.0
            pub.publish(twist)
            rate.sleep()

    elif command == "呼叫医生":
        rospy.loginfo("Moving backward to call doctor")
        
        # 转弯
        start_time = rospy.get_time()
        while rospy.get_time() - start_time < turn_duration and not rospy.is_shutdown():
            twist.linear.x = 0.0
            twist.angular.z = 0.5
            pub.publish(twist)
            rate.sleep()

        # 停留
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        rospy.sleep(stop_duration)
        
        # 向前移动
        start_time = rospy.get_time()
        while rospy.get_time() - start_time < move_duration and not rospy.is_shutdown():
            twist.linear.x = 0.5
            twist.angular.z = 0.0
            pub.publish(twist)
            rate.sleep()
        # 转弯
        start_time = rospy.get_time()
        while rospy.get_time() - start_time < turn_duration and not rospy.is_shutdown():
            twist.linear.x = 0.0
            twist.angular.z = 0.5
            pub.publish(twist)
            rate.sleep()

        # 停留
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        rospy.sleep(stop_duration)
        
        # 向前移动
        start_time = rospy.get_time()
        while rospy.get_time() - start_time < move_duration and not rospy.is_shutdown():
            twist.linear.x = 0.5
            twist.angular.z = 0.0
            pub.publish(twist)
            rate.sleep()
    
    else:
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        rospy.loginfo("Unknown command")
        pub.publish(twist)

def controller():
    rospy.init_node('controller_node')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('command_topic', String, callback, pub)
    rospy.spin()

if __name__ == '__main__':
    controller()
