#!/usr/bin/env python2
#coding=utf-8

import os
import scipy.io as sio
import rospy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry

def rflyVelCb(msg):
    global start_time, data
    t = msg.header.stamp.secs + msg.header.stamp.nsecs/1000000000.0
    if start_time == 0:
        start_time = t
    v = [t-start_time, msg.point.x, msg.point.y, msg.point.z]
    data["vel_obs"].append(v)

def rflyDistCb(msg):
    global start_time, data
    t = msg.header.stamp.secs + msg.header.stamp.nsecs/1000000000.0
    if start_time == 0:
        start_time = t
    p = [t-start_time, msg.point.x, msg.point.y, -msg.point.z]
    data["pos_obs"].append(p)

def airsimOdomCb(msg):
    global start_time, data, delta_time, file_name
    t = msg.header.stamp.secs + msg.header.stamp.nsecs/1000000000.0
    if start_time == 0:
        start_time = t
    v = [t-start_time, msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]
    data["vel_gt"].append(v)
    p = [t-start_time, msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
    data["pos_gt"].append(p)
    # 5s一存
    if t-start_time > delta_time:
        sio.savemat(file_name, data)
        delta_time += 5

def listener():
    rospy.init_node('result', anonymous=True)
    #Subscriber函数第一个参数是topic的名称，第二个参数是接受的数据类型 第三个参数是回调函数的名称
    rospy.Subscriber('airsim_node/Drone0/odom_local_ned', Odometry, airsimOdomCb)
    rospy.Subscriber('rfly/velocity', PointStamped, rflyVelCb)
    rospy.Subscriber('rfly/dist', PointStamped, rflyDistCb)
    rospy.spin()


global start_time, data, delta_time, file_name
file_name = "/home/zhenglong/guidance_ws/src/airsim_environment/matlab/data.mat"
data = {"vel_obs":[], "pos_obs":[], "vel_gt":[], "pos_gt":[]}
start_time = 0
delta_time = 5
listener()