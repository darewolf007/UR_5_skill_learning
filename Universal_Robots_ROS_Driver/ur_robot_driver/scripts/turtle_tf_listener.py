#!/usr/bin/env python
#-*-coding: utf-8 -*-
 
import rospy
import tf
import math
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge
import cv2 
import numpy as np
import time
import json
import os
time_tuple = time.localtime(time.time())
from std_srvs.srv import Trigger, TriggerResponse

 
class RecordROS():
    def __init__(self):
        self.cvbridge = CvBridge()
        self.count = 0
        self.is_save = False
        self.images = None
        self.ps_msg = None
        rospy.Subscriber('/tf',TFMessage,self.tf_callback,queue_size=2)
        rospy.Subscriber('/rgb/image_raw', Image, self.color_callback)
        rospy.sleep(1)
        rospy.wait_for_service("save_service")
        self.save_service_proxy = rospy.ServiceProxy("save_service", Trigger)

    def tf_callback(self,msg):
        self.count=self.count+1
        self.ps_msg = msg


    def color_callback(self,image_msg):
        image = self.cvbridge.imgmsg_to_cv2(image_msg,  desired_encoding='rgb8')

        self.images = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
        self.images = self.images[192:1344,550:1702]
        self.images = cv2.resize(self.images,(256,256))

    # def is_save_image():
    #     rospy.wait_for_service("my_service")
    #     my_service_proxy = rospy.ServiceProxy("my_service", Trigger)
    #     response = my_service_proxy()
    #     print(response.message)

if __name__ == '__main__':
    rospy.init_node('tf', anonymous = True)
    recordros = RecordROS() 
    rate = rospy.Rate(4)
    count = 0
    while not rospy.is_shutdown():
        count = count+1
        image = recordros.images
        # print(dir(recordros.ps_msg.transforms))
        # print(type(recordros.ps_msg))
        imagefile = '/home/yhx/Documents/catkin_ws/src/Universal_Robots_ROS_Driver/ur_robot_driver/scripts/data/wall/demo_99/image{}.png'.format(count)
        if recordros.save_service_proxy().success:
            cv2.imwrite(imagefile, image)
            with open('/home/yhx/Documents/catkin_ws/src/Universal_Robots_ROS_Driver/ur_robot_driver/scripts/data/wall/demo_99/tf_record.txt','a') as f:
                f.writelines("{}".format(recordros.ps_msg.transforms))
        rate.sleep()

# if __name__ == '__main__':
#     rospy.init_node('tf', anonymous = True)
#     recordros = RecordROS() 
#     rate = rospy.Rate(4)
#     while not rospy.is_shutdown():
#         with open('/home/yhx/Documents/catkin_ws/src/Universal_Robots_ROS_Driver/ur_robot_driver/scripts/data/wall/demo_0/tf_record.txt','a') as f:
#             f.writelines("{}".format(recordros.ps_msg.transforms))
#         rate.sleep()