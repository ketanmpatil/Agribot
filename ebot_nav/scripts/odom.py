#! /usr/bin/env python3

import rospy
from std_msgs.msg import Int16

odom = None

def odom_callback(data):
    global odom
    odom = data.data

def get_odom():
    rospy.init_node('odom_pub', anonymous=True)
    rospy.Subscriber("aruco_pub", Int16, odom_callback)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if odom != None:
            print(odom)
            break
        rate.sleep()

if __name__ == '__main__':
    get_odom()