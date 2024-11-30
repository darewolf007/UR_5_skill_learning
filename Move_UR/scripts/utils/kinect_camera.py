# -*- coding: utf-8 -*-
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
# ROS库
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import CameraInfo, Image
import rospy
# import cv_bridge
import ros_numpy

import pdb
# 自建库
# from thmp_solution.msg import PredPose


color_R = np.array([[0.999985, 0.00392864, -0.00372753],
                    [-0.00352813, 0.994782, 0.10196],
                    [0.00410865, -0.101946, 0.994781]])
color_T = np.array([-32.0521, -1.90638, 3.84897]) * 1e-3


class KinectDK(object):
    def __init__(self):
        self.queue_hbody = Queue.Queue(3)
        self.queue_color = Queue.Queue(3)
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
        while self.rgb_info is None or self.depth_info is None:
            time.sleep(0.1)
            t += 0.1
            if t >= timeout:
                raise Queue.Empty

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


class Thmp(object):
    """计划废弃de函数"""
    def __init__(self):
        import Queue
        self.thmp_sub = rospy.Subscriber('/thmp/predictions', PredPose, self.thmp_callback)
        self.queue_thmp = Queue.Queue(2)

    def thmp_callback(self, msg):
        if not self.queue_thmp.full():
            self.queue_thmp.put(msg)

