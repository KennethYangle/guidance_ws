#!/usr/bin/env python2
#coding=utf-8

import rospy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry

def rflyVelCb(msg):
    pass

def rflyDistCb(msg):
    pass

def airsimOdomCb(msg):
    pass

def listener():
    rospy.init_node('result', anonymous=True)
    #Subscriber函数第一个参数是topic的名称，第二个参数是接受的数据类型 第三个参数是回调函数的名称
    rospy.Subscriber('airsim_node/Drone0/odom_local_ned', Odometry, airsimOdomCb)
    rospy.Subscriber('rfly/velocity', PointStamped, rflyVelCb)
    rospy.Subscriber('rfly/dist', PointStamped, rflyDistCb)
    rospy.spin()

if __name__ == '__main__':
    listener()