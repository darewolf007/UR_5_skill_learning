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
time_tuple = time.localtime(time.time())
import ros_numpy
import Queue
class KinectDK(object):
    def __init__(self):
        import time
        import io
        import sys
        try:
            import Queue
        except:
            import queue as Queue

        # 第三方库
        import cv2
        import numpy as np
        import scipy.io as scio
        # ROS库
        from visualization_msgs.msg import MarkerArray
        from sensor_msgs.msg import CameraInfo, Image
        import rospy
        # import cv_bridge
        import ros_numpy

        import pdb
        self.queue_hbody = Queue.Queue(3)
        self.queue_color = Queue.Queue(1)
        self.queue_depth = Queue.Queue(3)
        self.queue_left_hand = Queue.Queue(3)
        # self.bridge = cv_bridge.CvBridge()
        # pdb.set_trace()
        self.rgb_info = None
        self.depth_info = None
        print('*'*20)
        self.hbody_sub = rospy.Subscriber("/body_tracking_data", MarkerArray, self.hbody_callback)
        # pdb.set_trace()
        self.left_hand_sub = rospy.Subscriber('/cameras/left_hand_camera/image', Image, self.left_hand_callback)
        
        # pdb.set_trace()
        self.color_sub = rospy.Subscriber('/rgb/image_raw', Image, self.color_callback)
        self.depth_sub = rospy.Subscriber('/depth_to_rgb/image_raw', Image, self.depth_callback)
        self.rgbinfo_sub = rospy.Subscriber('/rgb/camera_info', CameraInfo, self.rgbinfo_callback)
        self.depthinfo_sub = rospy.Subscriber('/depth/camera_info', CameraInfo, self.depthinfo_callback)
        # pdb.set_trace()
        self.wait_init()

    def wait_init(self, timeout=10.):
        t = 0.
        print("5")
        while self.rgb_info is None or self.depth_info is None:
            time.sleep(0.1)
            t += 0.1
            if t >= timeout:
                raise Queue.Empty
        print("7")

    def rgbinfo_callback(self, camera_info):
        self.rgb_info = camera_info
        self.rgbinfo_sub.unregister()

    def depthinfo_callback(self, camera_info):
        self.depth_info = camera_info
        self.depthinfo_sub.unregister()

    def hbody_callback(self, marker_msg):
        if self.queue_hbody.full():
            self.queue_hbody.get_nowait()
        self.queue_hbody.put(marker_msg.markers)

    def color_callback(self, image_msg):  # bgra
        # cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
        # pdb.set_trace()
        # if self.queue_color.full():
        #     self.queue_color.get()
        # self.queue_color.put(np.asarray(cv_image))
        cv_image = ros_numpy.numpify(image_msg)
        if self.queue_color.full():
            self.queue_color.get()
        self.queue_color.put(np.asarray(cv_image))

    def left_hand_callback(self, image_msg):  # left hand image
        # cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
        # if self.queue_left_hand.full():
        #     self.queue_left_hand.get()
        # self.queue_left_hand.put(np.asarray(cv_image))
        cv_image = ros_numpy.numpify(image_msg)
        if self.queue_left_hand.full():
            self.queue_left_hand.get()
        self.queue_left_hand.put(np.asarray(cv_image))
        

    def depth_callback(self, image_msg):  # float32
        # cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
        # if self.queue_depth.full():
        #     self.queue_depth.get()
        # self.queue_depth.put(np.asarray(cv_image))
        cv_image = ros_numpy.numpify(image_msg)
        if self.queue_depth.full():
            self.queue_depth.get()
        self.queue_depth.put(np.asarray(cv_image))

    def release(self):
        print("kinect_release")
        self.hbody_sub.unregister()
        self.color_sub.unregister()
        self.depth_sub.unregister()
        self.left_hand_sub.unregister()

class RecordROS():
    def __init__(self):
        # rospy.init_node('tf', log_level=rospy.DEBUG)
        self.cvbridge = CvBridge()
        self.count = 0

        rospy.init_node('tf', anonymous = True)
        self.image = KinectDK() 
        # 唯一的节点名 日志级别为DEBUG 等级排序为 DEBUG INFO WARN ERROR FATAL

    def tf_callback(self,ps_msg):
        self.count=self.count+1
        print(ps_msg)   
        # print(self.count+1)  
        img_color = self.image.queue_color.get(timeout=1.0)
        cv2.imshow("test", img_color)  
        imagefile = '/home/yhx/Documents/catkin_ws/src/Universal_Robots_ROS_Driver/ur_robot_driver/scripts/image/image{}.png'.format(self.count)
        cv2.imwrite(imagefile, img_color)
        # color_sub = rospy.Subscriber('/rgb/image_raw', Image, self.color_callback,queue_size=1)
        # rate.sleep()
        # rospy.sleep(0.5)

    def color_callback(self,image_msg):
        image = self.cvbridge.imgmsg_to_cv2(image_msg,  desired_encoding='rgb8')
        image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)    
        cv2.imshow("test", image)    

        # imagefile = '/home/yhx/Documents/catkin_ws/src/Universal_Robots_ROS_Driver/ur_robot_driver/scripts/image/image{}.png'.format(self.count)
        # cv2.imwrite(imagefile, image)


 
        
if __name__ == '__main__':
    recordros = RecordROS()

    # image = KinectDK() 
    # img_color = kinect_dk.queue_color.get(timeout=10.0)
    # # listener = tf.TransformListener()
    rate = rospy.Rate(2)
    # while not rospy.is_shutdown():
        # color_sub = rospy.Subscriber('/rgb/image_raw', Image, recordros.color_callback,queue_size=1)
    rospy.Subscriber('/tf',TFMessage,recordros.tf_callback,queue_size=1)
    rospy.spin()