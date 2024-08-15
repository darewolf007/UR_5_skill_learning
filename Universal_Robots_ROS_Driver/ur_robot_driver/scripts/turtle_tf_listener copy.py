#!/usr/bin/env python
#-*-coding: utf-8 -*-
 
import rospy
from geometry_msgs.msg import Twist
import tf
import math
from tools.camera import KinectDK
from tf2_msgs.msg import TFMessage
class RecordROS():
    def __init__(self):
        # 唯一的节点名 日志级别为DEBUG 等级排序为 DEBUG INFO WARN ERROR FATAL
        self.kinect_dk = KinectDK()
        rospy.init_node('tf', log_level=rospy.DEBUG)
    def tf_callback(self, msg):
        print(msg)
        img_color = self.kinect_dk.queue_color.get(timeout=10.0)
        rospy.Subscriber('/rgb/image_raw',TFMessage,callback)
    def shutdown(self):
        print "shut down node..."

    
        
if __name__ == '__main__':
    recordros = RecordROS()
    rospy.Rate(2)
    print(2)
    while not rospy.is_shutdown():
        print(1)
        pub = rospy.Subscriber('tf',TFMessage,tf_callback)
        rospy.spin()                 
        rospy.sleep(1)
    