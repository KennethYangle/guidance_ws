#!/usr/bin/env python2
#coding=utf-8

import rospy
#导入自定义的数据类型
from airsim_environment.msg import VelCmd

def talker():
    #Publisher 函数第一个参数是话题名称，第二个参数 数据类型，现在就是我们定义的msg 最后一个是缓冲区的大小
    #queue_size: None（不建议）  #这将设置为阻塞式同步收发模式！
    #queue_size: 0（不建议）#这将设置为无限缓冲区模式，很危险！
    #queue_size: 10 or more  #一般情况下，设为10 。queue_size太大了会导致数据延迟不同步。
    pub = rospy.Publisher('airsim_node/Drone0/vel_cmd_body_frame', VelCmd , queue_size=10)
    rospy.init_node('trace', anonymous=True)
    #更新频率是1hz
    rate = rospy.Rate(50) 
    msg = VelCmd()
    # # line
    # msg.twist.linear.x = 1
    # circle
    msg.twist.linear.x = 1
    msg.twist.linear.z = -0.2
    msg.twist.angular.z = 0.03
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    talker()