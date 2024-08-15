#!/usr/bin/env python
#coding=utf-8
from re import S
from shutil import move
from turtle import left
from scipy import array

from torch import grid_sampler

from pickle import FALSE
import numpy as np
import os, time
import cv2
import csv
import pandas as pd
import scipy.io as scio
import transforms3d
from transforms3d.quaternions import quat2mat, mat2quat, quat2axangle
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,)
from std_msgs.msg import Header
# import cv_bridge
# from cv_bridge import CvBridge, CvBridgeError
import ros_numpy
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import Image, CameraInfo
try:
    import Queue
except:
    import queue as Queue

from prob1.srv import VmrnDetection, VmrnRelation

import matplotlib
matplotlib.use('Agg')


class KinectDK(object):
    def __init__(self):
        self.queue_hbody = Queue.Queue(3)
        self.queue_color = Queue.Queue(3)
        self.queue_depth = Queue.Queue(3)
        self.queue_left_hand = Queue.Queue(3)
        # self.bridge = cv_bridge.CvBridge()
        self.rgb_info = None
        self.depth_info = None
        self.hbody_sub = rospy.Subscriber("/body_tracking_data", MarkerArray, self.hbody_callback)
        self.left_hand_sub = rospy.Subscriber('/cameras/left_hand_camera/image', Image, self.left_hand_callback)
        self.color_sub = rospy.Subscriber('/rgb/image_raw', Image, self.color_callback)
        self.depth_sub = rospy.Subscriber('/depth_to_rgb/image_raw', Image, self.depth_callback)
        self.rgbinfo_sub = rospy.Subscriber('/rgb/camera_info', CameraInfo, self.rgbinfo_callback)
        self.depthinfo_sub = rospy.Subscriber('/depth/camera_info', CameraInfo, self.depthinfo_callback)
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

def ik_solve(limb, pos, orient):
    # ~ rospy.init_node("rsdk_ik_service_client")
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    # print "iksvc: ", iksvc
    # print "ikreq: ", ikreq
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        str(limb): PoseStamped(header=hdr,
                               pose=Pose(position=pos, orientation=orient))}

    ikreq.pose_stamp.append(poses[limb])
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1
    if (resp.isValid[0]):
        print("SUCCESS - Valid Joint Solution Found:")
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        # print limb_joints
        return limb_joints
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
    return -1



Left_Init_Joint_Angles=[0.07478156340941391, -0.9077331312311936, -0.9364952710040451, 1.650179832567734, 0.5852136705782853, 1.1347622878382349, -0.29797576804674164]
Right_Init_Joint_Angles= []
Left_Joint_angle_keys = ['left_s0','left_s1','left_e0','left_e1','left_w0','left_w1','left_w2']
Right_Joint_angle_keys = ['right_s0','right_s1','right_e0','right_e1','right_w0','right_w1','right_w2']
class Baxter_Controller(object):
    def __init__(self):
        rospy.init_node('baxter_grasp', anonymous=True)
        rospy.loginfo("Enabling robot... ")
        self.rs = baxter_interface.RobotEnable()
        self.rs.enable()
        arm= baxter_interface.Limb('left')
        print("endpose position:",arm.endpoint_pose()['position'])
        

    def get_init_arm_joint(self):
        print('get limb init joint angles......')
        arm_left = baxter_interface.Limb('left')
        # print('left arm endpose:', arm_left.endpoint_pose())
        angles = arm_left.joint_angles()
        for key in Left_Joint_angle_keys:
            Left_Init_Joint_Angles.append(angles[key])
        print('left init joint angles', Left_Init_Joint_Angles)
        arm_right = baxter_interface.Limb('right')
        # print('right arm endpose:', arm_right.endpoint_pose())
        angles = arm_right.joint_angles()
        for key in Right_Joint_angle_keys:
            Right_Init_Joint_Angles.append(angles[key])
        print('right init joint angles', Right_Init_Joint_Angles)

    def get_init_v(self):
        print('get limb init joint angles......')
        arm_left = baxter_interface.Limb('left')
        arm_left.set_joint_velocities(cmd)

    def reset_to_standerder_ori(self, limb='left'):
        arm = baxter_interface.Limb(limb)
        pose = arm.endpoint_pose()['position']
        self.move_limb_to_point(point=pose, ori=[0,1,0,0],limb=limb)


    def move_limb_to_neutral(self, limb='left', vel=1.0):
        arm = baxter_interface.Limb(limb)
        arm.set_joint_position_speed(vel)
        
        if limb == 'right':
            angle_list = Right_Init_Joint_Angles
        elif limb == 'left':
            angle_list = Left_Init_Joint_Angles
        #print(arm.joint_names())
        angles = dict(zip(arm.joint_names(), angle_list))
        flag = arm.move_to_joint_positions(angles)
        rospy.sleep(0.1)
        self.open_gripper(limb)
        rospy.sleep(0.1)

    def open_gripper(self, limb="left"):
        gripper = baxter_interface.Gripper(limb, CHECK_VERSION)
        rospy.sleep(0.2)
        gripper.open()
        rospy.sleep(0.2)

    def close_gripper(self, limb="left"):
        gripper = baxter_interface.Gripper(limb, CHECK_VERSION)
        rospy.sleep(0.2)
        gripper.close()
        rospy.sleep(0.2)

    def move_limb_to_point(self, point, ori=[0,1,0,0], limb='left', vel=2.0):
        limbhandle = baxter_interface.Limb(limb)
        limbhandle.set_joint_position_speed(vel)
        loc = Point(point[0], point[1], point[2])
        # w x y z
        ang = Quaternion(ori[0], ori[1], ori[2], ori[3])
        limbjoints = ik_solve(limb, loc, ang)
        if limbjoints == -1:
            return -1
        limbhandle.move_to_joint_positions(limbjoints)
        return 1


class DemoGnerateor(object):
    def __init__(self):
        self.RoController = Baxter_Controller()
        self.kinect_dk = KinectDK()
        self.K, self.D, self.map1, self.map2 = self.get_camera_info()
        D = scio.loadmat("/home/xj/Documents/catkin_ws/calibrate_para/extpara.mat")
        self.T, self.R = D["T"].reshape([3]), D["R"].reshape([3, 3])
        # self.RoController.move_limb_to_neutral('left')
        # self.RoController.open_gripper()
        # index = 13
        # img_path = '/home/xj/Documents/catkin_ws/src/prob1/scripts/vmrn/images/example_'+str(index) +'.jpg'
        # color = self.kinect_dk.queue_color.get(timeout=5.0)
        # depth = self.kinect_dk.queue_depth.get(timeout=5.0)
        # color_remap = self.remap_img(color, 'rgb')
        # depth_remap = self.remap_img(depth, 'depth')
        # cv2.imwrite(img_path, color_remap)
        # rospy.sleep(2)
        # res = self.get_vmrn_results(index)

    def exect_grasp(self):
    
        self.RoController.move_limb_to_neutral(limb='left')
        self.RoController.open_gripper(limb='left')
        # self.RoController.move_limb_to_neutral(limb='right')
        index = 0
        while True:
            print('index:', index)
            img_path = '/home/xj/Documents/catkin_ws/src/prob1/scripts/vmrn/images/example_'+str(index) +'.jpg'
            color = self.kinect_dk.queue_color.get(timeout=5.0)
            depth = self.kinect_dk.queue_depth.get(timeout=5.0)
            color_remap = self.remap_img(color, 'rgb')
            depth_remap = self.remap_img(depth, 'depth')

            cv2.imwrite(img_path, color_remap)#!!!!!!!!!!!!
            rospy.sleep(2)
            res_grasp, res_relation = self.get_vmrn_results(index)
            print("grasp",res_grasp)
            print("current_target",res_relation[0].current_target.id)
            if res_relation[0].current_target.id is not -1:
                if len(res_grasp) >0:
                # for result in res_grasp:
                #     if result.obj_cls == "grenade":
                #         grasps = result.obj_grasp.grasp
                #         uv = [int(grasps[0]), int(grasps[1]), 1]
                #         depth = depth_remap[uv[1]][uv[0]].astype(np.float)
                #         grasp_center = self.transform_uv_to_xy(self.R, self.T, self.K, uv, depth)
                #         ori = self.TransAngle2Ori(grasps[2])
                #         print(ori,  grasp_center)
                #         offset = np.array([0, 0 ,0.1])
                #         self.RoController.move_limb_to_point(point=grasp_center + offset, ori=ori,limb='left')
                #         self.RoController.move_limb_to_point(point = grasp_center + np.array([0,0,-0.026]), ori=ori,limb='left')
                #         self.RoController.close_gripper()

                #         putdown_position = [0.65, 0.73, 0.1]
                #         self.RoController.move_limb_to_point(point=grasp_center+offset, ori=[0,1,0,0],limb='left')
                #         self.RoController.move_limb_to_point(point=putdown_position,ori=[0,1,0,0],limb='left')
                #         self.RoController.move_limb_to_point(point=np.array(putdown_position)-np.array([0,0,0.1]),ori=[0,1,0,0],limb='left')
                #         self.RoController.open_gripper()
                #         self.RoController.move_limb_to_point(point=putdown_position,ori=[0,1,0,0],limb='left')
                #         break
                    id = res_relation[0].current_target.id
                    grasps = res_grasp[id].obj_grasp.grasp
                    uv = [int(grasps[0]), int(grasps[1]), 1]
                    print("uv:",uv)
                    depth = depth_remap[uv[1]][uv[0]].astype(np.float)
                    grasp_center = self.transform_uv_to_xy(self.R, self.T, self.K, uv, depth)
                    ori = self.TransAngle2Ori(grasps[2])
                    #print("depth:",depth)
                    print()
                    print("ori,grasp_center")
                    print(ori,  grasp_center)
                    offset = np.array([0, 0 ,0.1])
                    ########bbox center replace grasp center  ####begin
                    # bbx = res_grasp[id].obj_bbox.xyxy#[x,y,x,y]
                    # lenth = abs(bbx[2] - bbx[0])
                    # width = abs(bbx[3] - bbx[1])
                    # print("lenth,width,delta",lenth,width,abs(lenth-width))
                    # epsilon = 10#????????????????
                    # if abs(lenth-width) < epsilon and res_grasp[id].obj_cls == "grenade":
                        
                    #    uv = [int(np.min([bbx[0], bbx[2]]) + lenth/2), int(np.min([bbx[1], bbx[3]]) + width/2), 1]
                    #    print("new_uv",uv)
                    ########end

                    depth = depth_remap[uv[1]][uv[0]].astype(np.float)
                    grasp_center = self.transform_uv_to_xy(self.R, self.T, self.K, uv, depth)
                    ori = self.TransAngle2Ori(grasps[2])
                    #print("depth:",depth)
                    print()
                    print("ori,grasp_center")
                    print(ori,  grasp_center)
                    offset = np.array([0, 0, 0.1])#[0, -0.09 ,0.1])

                    ######## depth--->z begin
                    # bbox = res_grasp[id].obj_bbox.xyxy
                    # #print(np.min((bbox[0],bbox[1],bbox[2],bbox[3])),bbox[0],bbox[1],bbox[2],bbox[3])
                    # depth_uv_area = depth_remap[int(np.min((bbox[1],bbox[3]))):int(np.max((bbox[1],bbox[3]))),int(np.min((bbox[0],bbox[2]))):int(np.max((bbox[0],bbox[2])))].astype(np.float)
                    # #print("shape:",depth_uv_area.shape)
                    # #print("shape",depth_remap.shape)
                    # #print(int(np.min((bbox[1],bbox[3]))),int(np.max((bbox[1],bbox[3]))))
                    # #print(depth_uv_area)
                    # record(depth_uv_area)
                    # select_depth =np.min(depth_uv_area[np.nonzero(depth_uv_area)])
                    # depth_uv = np.where(depth_uv_area == select_depth)
                    # #print("depth_uv:",depth_uv)
                    # #print(depth_uv[0])
                    # uv_d = [int(np.min((bbox[0],bbox[2])))+depth_uv[1][0], int(np.min((bbox[1],bbox[3])))+depth_uv[0][0], 1]
                    # #print("compare_uv:",uv,uv_d)
                    # grasp_center_d = self.transform_uv_to_xy(self.R, self.T, self.K, uv_d, select_depth)
                    # #print("min_depth:", select_depth)
                    # #print("compare:old&new",grasp_center, grasp_center_d)
                    # #print("depth_remap_shape:",depth_remap.shape)
                    # #grasp_center[2] = grasp_center_d[2]#!!!!!!!!
                    # #print(grasp_center)
                    ######## depth--->z end

                    # self.RoController.move_limb_to_point(point=grasp_center + np.array([-0.02,0.04,-0.060]) + offset, ori=ori,limb='left')
                    self.RoController.move_limb_to_point(point=grasp_center + np.array([-0.01,0.006,0.00]) + offset, ori=ori,limb='left')
                    self.RoController.move_limb_to_point(point = grasp_center + np.array([-0.01,0.006,-0.040]), ori=ori,limb='left')
                    self.RoController.close_gripper()

                    putdown_position = [0.65, 0.73, 0.1]
                    self.RoController.move_limb_to_point(point=grasp_center+offset+ np.array([0.0,0.01,0.00]), ori=[0,1,0,0],limb='left')
                    self.RoController.move_limb_to_point(point=putdown_position,ori=[0,1,0,0],limb='left')
                    self.RoController.move_limb_to_point(point=np.array(putdown_position)-np.array([0,0,0.1]),ori=[0,1,0,0],limb='left')
                    self.RoController.open_gripper()
                    self.RoController.move_limb_to_point(point=putdown_position,ori=[0,1,0,0],limb='left')
            else:
                break
            index += 1

    def exec_grasp2(self):
        # defined_grasp_pos=[[0.6374056478440194, 0.22710705925986852,-0.03982011845755175],[0.6747210375230649,0.01660560468072707, -0.045489854675021334],[0.65939465798628, 0.09742009278310554, -0.04309388263088146]]
        defined_grasp_pos = [[0.5787526151464915, -0.10352942326430684,-0.12131939067758807]]
        
        defined_grasp_ori = [10]

        for i in range(len(defined_grasp_pos)):
            
            self.RoController.move_limb_to_neutral(limb='left')
            offset = np.array([0, 0 ,0.1])
            ori = self.TransAngle2Ori(defined_grasp_ori[i])
            self.RoController.move_limb_to_point(point=np.array(defined_grasp_pos[i])+offset, ori=ori, limb='left')
            self.RoController.move_limb_to_point(point=np.array(defined_grasp_pos[i])+np.array([-0.012,0,-0.008]), ori=ori, limb='left')
            self.RoController.close_gripper()
            self.RoController.move_limb_to_point(point=np.array(defined_grasp_pos[i])+offset, ori=[0,1,0,0], limb='left')
            # putdown_position = [0.65, 0.73, 0.1] #0.73
            putdown_position = [0.65, 0.56, 0.1]#grenade
            # putdown_position = [0.41, 0.51, 0.1]#other
            self.RoController.move_limb_to_point(point=putdown_position,ori=[0,1,0,0],limb='left')
            self.RoController.move_limb_to_point(point=np.array(putdown_position)-np.array([0,0,0.1]),ori=[0,1,0,0],limb='left')
            self.RoController.open_gripper()
            


    def transform_uv_to_xy(self, R, T, K, uv, depth):
        xy_reshape = np.array(uv).reshape((3,1))
        T_reshape = np.array(T).reshape((3,1))
        K_inv = np.linalg.inv(K)

        print(K_inv)
        print(depth)
        print('--------------------------------')

        T_diff = depth*np.dot(K_inv, xy_reshape)
        
        world_xyz = np.dot(R, T_diff)+ T_reshape

        #s = np.einsum("ab,nb->na", color_R, xy_reshape) + color_T
        #world_xyz = np.einsum("ab,nb->na", R, xy_reshape) + T
        return world_xyz.flatten()

    def get_camera_info(self):
        img_color = self.kinect_dk.queue_color.get(timeout=10.0)
        K = np.asarray(self.kinect_dk.rgb_info.K).reshape(3, 3) ## 相机内参
        D = np.asarray(self.kinect_dk.rgb_info.D)  ## 畸变系数
        size = img_color.shape[:2][::-1]
        map1, map2 = cv2.initUndistortRectifyMap(K, D, None, None, size, cv2.CV_32FC1)
        return K, D, map1, map2

    def remap_img(self, img, type):
        if type == 'rgb':
            img_remap = cv2.remap(img, self.map1, self.map2, cv2.INTER_CUBIC)
        else:
            img_remap = cv2.remap(img, self.map1, self.map2, cv2.INTER_NEAREST)
        return img_remap

    def TransAngle2Ori(self, angle_in_dgree):
        from transforms3d.euler import euler2quat, quat2euler
        quat = np.array([0., 0, 1, 0])
        euler = np.asarray(quat2euler(quat, 'sxyz'))
        euler[2] += angle_in_dgree * np.pi / 180.
        while euler[2] < -np.pi:
            euler[2] += np.pi * 2
        while euler[2] > np.pi:
            euler[2] -= np.pi * 2
        quat = euler2quat(euler[0], euler[1], euler[2], 'sxyz')
        return (quat[1], quat[2], quat[3], quat[0])


    def get_vmrn_results(self, index):
        rospy.wait_for_service('vmrn_detection')
        print('vmrn detection.......')
        try:
            obj_client_grasp = rospy.ServiceProxy('vmrn_detection',VmrnDetection)
            obj_client_relation = rospy.ServiceProxy('vmrn_relation',VmrnRelation)
            response_grasp = obj_client_grasp.call(True, index)
            response_relation = obj_client_relation.call(True, index)
            return response_grasp.results, response_relation.results
        except rospy.ServiceException as e:
            rospy.loginfo("service call failed:%s"%e)

def record(matrix):
    file = open("/home/xj/Documents/catkin_ws/src/prob1/scripts/vmrn/result.txt", 'w').close()
    with open('/home/xj/Documents/catkin_ws/src/prob1/scripts/vmrn/result.txt','w') as f:
   
        for x in range(matrix.shape[0]):
            for y in range(matrix.shape[1]):
                #print(x,y)
                f.write(str(matrix[x][y])+' ')
                n = matrix.shape[1]
                if y == n-1:
                    f.write('\n')
                else:
                    continue
if __name__ == "__main__":
    generator = DemoGnerateor()
    flag= True
    if flag:
        shijiao = True
        if shijiao:
            generator.exec_grasp2()
        else:
            generator.exect_grasp()