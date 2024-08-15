#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import Image
import cv2 
import numpy as np
from cv_bridge import CvBridge
import time
time_tuple = time.localtime(time.time())


class Save_image():
    def __init__(self):
        self.count = 0
        self.cvbridge = CvBridge()
    
    def message(self, data):
        print(data.encoding)

    
    def save_image(self, data):

        image = self.cvbridge.imgmsg_to_cv2(data,  desired_encoding='rgb8')
        image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)        
        imagefile = '/home/xj/Documents/catkin_ws/src/Universal_Robots_ROS_Driver/ur_robot_driver/scripts/image/image{}.{}.{}.{}.{}.{}.png'.format(time_tuple[0],time_tuple[1],time_tuple[2],time_tuple[3],time_tuple[4],time_tuple[5])
        cv2.imwrite(imagefile, image)


    
    def save_depth(self, data):
        
        depth = self.cvbridge.imgmsg_to_cv2(data,  desired_encoding='16UC1')
        depthfile = '/home/xj/Documents/catkin_ws/src/Universal_Robots_ROS_Driver/ur_robot_driver/scripts/depth/depth{}.{}.{}.{}.{}.{}.png'.format(time_tuple[0],time_tuple[1],time_tuple[2],time_tuple[3],time_tuple[4],time_tuple[5])
        cv2.imwrite(depthfile, depth)
        self.count += 1




# try:
a = Save_image()
rospy.init_node('save_image', anonymous = True)
a.count = 0
while a.count < 1:
    rospy.Subscriber("/camera/color/image_raw", 
                    Image, 
                    a.save_image)  
    rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", 
                    Image, 
                    a.save_depth)  
print("keep successfully !")


# a.count = 0
# while a.count < 1:
#     rospy.Subscriber("/camera/color/image_raw", 
#                     Image, 
#                     a.save_image)  
#     rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", 
#                     Image, 
#                     a.save_depth)  
# print("keep successfully !")
# rospy.sleep(2)

# a.count = 0
# while a.count < 1:
#     rospy.Subscriber("/camera/color/image_raw", 
#                     Image, 
#                     a.save_image)  
#     rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", 
#                     Image, 
#                     a.save_depth)  
# print("keep successfully !")
    

# except rospy.ROSInterruptException:
#     pass
