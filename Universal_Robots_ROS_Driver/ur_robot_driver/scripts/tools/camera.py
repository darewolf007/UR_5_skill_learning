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
import scipy.io as scio
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


class Thmp(object):
    """计划废弃de函数"""
    def __init__(self):
        import Queue
        self.thmp_sub = rospy.Subscriber('/thmp/predictions', PredPose, self.thmp_callback)
        self.queue_thmp = Queue.Queue(2)

    def thmp_callback(self, msg):
        if not self.queue_thmp.full():
            self.queue_thmp.put(msg)


class IOPredictor(object):
    """传感器接口"""
    def __init__(self, affine=None):
        # 若指定相机外参affine = (T, R)，得到数据后会从相机坐标系转换到世界坐标系
        self.pred_sub = rospy.Subscriber("/thmp/predictions", PredPose, self.pred_callback, queue_size=2)
        self.affine = affine
        self.stamp = rospy.Time.now()
        self.isrej = True
        self.data = None

    def pred_callback(self, msg):
        # 后处理 + 仿射变换（转换到世界坐标系） =》 data["current"] (18,3), data["pred"] (30, 18, 3), data["stddev"] (30, 18, 3)
        # 保存在 self.data
        self.stamp = msg.header.stamp
        self.isrej = msg.isReject
        if len(msg.data) > 0:
            with io.BytesIO(msg.data) as f:
                data = scio.loadmat(f)
            # 后处理
            data["current"][..., 1:, :] += data["current"][..., :1, :]
            if "pred" in data:
                data["pred"][..., 1:, :] += data["pred"][..., :1, :]
            # 仿射变换
            if self.affine is not None:
                T, R = self.affine
                data["current"] = data["current"].reshape(-1, 3)
                data["current"] = np.einsum("ab,nb->na", color_R, data["current"]) + color_T
                data["current"] = np.einsum("ab,nb->na", R, data["current"]) + T
                if "pred" in data:
                    data["pred"] = data["pred"].reshape(-1, 3)
                    data["pred"] = np.einsum("ab,nb->na", color_R, data["pred"]) + color_T
                    data["pred"] = np.einsum("ab,nb->na", R, data["pred"]) + T
            # reshape
            data["current"] = data["current"].reshape(18, 3)
            if "pred" in data:
                data["pred"] = data["pred"].reshape(-1, 18, 3)
                data["stddev"] = data["stddev"].reshape(-1, 18)
            self.data = data
