#!/usr/bin/env python2
 
from __future__ import print_function

import sys
import rospy
from hh import get_new_pos
import numpy as np
import os
from math import pi
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import Constraints, JointConstraint
## END_SUB_TUTORIAL
import ipdb
from cv_bridge import CvBridge
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import CameraInfo, Image
# from PIL import Image
from Move_UR.srv import ddt,ddtRequest
import cv2
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from controller_manager_msgs.srv import LoadControllerRequest, LoadController
from controller_manager_msgs.srv import ListControllers, ListControllersRequest
import geometry_msgs.msg as geometry_msgs
from cartesian_control_msgs.msg import (
    FollowCartesianTrajectoryAction,
    FollowCartesianTrajectoryGoal,
    CartesianTrajectoryPoint,
)
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
import rospy
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from time import sleep
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, JointState
from aruco_msgs.msg import MarkerArray
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Bool, UInt32MultiArray
import os
from std_srvs.srv import SetBool, SetBoolResponse
import threading
if sys.version_info[0] < 3:
    input = raw_input

class Data_Collection:
    def __init__(self, init_node = True, path_id = 0):
        if init_node:
            rospy.init_node("data_collection")
        self.cvbridge = CvBridge()
        self.marker_list = []
        self.marker_dict = {}
        self.ur_joint_angle = None
        self.ur_endeffector_position = None
        self.scene_rgb_images = None
        self.scene_depth_images = None
        self.aruco_image = None
        self.gripper_state = 0
        self.marker_dict_collect = []
        self.ur_joint_angle_collect = []
        self.ur_endeffector_position_collect = []
        self.gripper_state_collect = []
        self.safe_lock = threading.Lock()
        self.aruco_markers_sub = rospy.Subscriber('/aruco_marker_publisher/markers_list', UInt32MultiArray, self.collect_aruco_markers)
        self.aruco_image_sub = rospy.Subscriber('/aruco_marker_publisher/result', Image, self.collect_aruco_image)
        self.aruco_result_sub = rospy.Subscriber('/aruco_marker_publisher/markers', MarkerArray, self.collect_aruco_results)
        self.color_sub = rospy.Subscriber('/rgb/image_raw', Image, self.collect_scene_rgb_image)
        # self.depth_sub = rospy.Subscriber('/depth_to_rgb/image_raw', Image, self.collect_scene_depth_image)
        self.ur_joint_sub = rospy.Subscriber('/joint_states', JointState, self.collect_UR_joint_angle)
        self.ur_endeffector_sub = rospy.Subscriber('/tf', TFMessage, self.collect_UR_endeffector_position)
        self.ur_gripper_sub = rospy.Subscriber('/ur_gripper', Bool, self.collect_gripper_state)
        self.record_num = 0
        base_data_path = "/home/yhx/shw/src/Dataset_Collection/"
        self.traj_directory_name = base_data_path + str(self.count_dirs_in_directory(base_data_path) +1)
        os.mkdir(self.traj_directory_name)
        os.mkdir(self.traj_directory_name + '/scene_rgb_image')
        os.mkdir(self.traj_directory_name + '/marker_result_image')
        os.mkdir(self.traj_directory_name + '/traj')
        os.mkdir(self.traj_directory_name + '/marker')
        
    def count_dirs_in_directory(self, path):
        count = sum(os.path.isdir(os.path.join(path, name)) for name in os.listdir(path))
        return count
        
    def collect_aruco_markers(self, marker_list):
        self.marker_list = marker_list.data
    
    def collect_aruco_image(self, image_msg):
        image = self.cvbridge.imgmsg_to_cv2(image_msg,  desired_encoding='rgb8')
        self.aruco_image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)

    def collect_aruco_results(self, marker_array):
        with self.safe_lock:
            self.marker_dict = {}
            for marker in marker_array.markers:
                marker_id = marker.id
                marker_pose = marker.pose.pose
                translation = marker_pose.position
                rotation = marker_pose.orientation
                translation_data = np.array([translation.x, translation.y, translation.z])
                rotation_data = np.array([rotation.x, rotation.y, rotation.z, rotation.w])
                self.marker_dict[marker_id] = np.concatenate((translation_data, rotation_data))
        
    def collect_UR_joint_angle(self, joint_state):
        self.ur_joint_angle = np.array(joint_state.position)
    
    def collect_UR_endeffector_position(self, tf_message):
        for transform in tf_message.transforms:
                if transform.child_frame_id == "tool0_controller":
                    timestamp = transform.header.stamp.to_sec()
                    frame_id = transform.header.frame_id
                    child_frame_id = transform.child_frame_id
                    translation = transform.transform.translation
                    rotation = transform.transform.rotation
                    translation_data = np.array([translation.x, translation.y, translation.z])
                    rotation_data = np.array([rotation.x, rotation.y, rotation.z, rotation.w])
                    self.ur_endeffector_position = np.concatenate((translation_data, rotation_data))
        
    def collect_scene_rgb_image(self, image_msg):
        image = self.cvbridge.imgmsg_to_cv2(image_msg,  desired_encoding='rgb8')
        self.scene_rgb_images = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
        
    def collect_scene_depth_image(self, image_msg):
        image = self.cvbridge.imgmsg_to_cv2(image_msg,  desired_encoding='rgb8')
        self.scene_depth_images = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
    
    def collect_gripper_state(self, gripper_state):
        if gripper_state.data:
            self.gripper_state = 1
        else:
            self.gripper_state = 0
        
    def record(self):
        with self.safe_lock:         
            if len(list(self.marker_dict.keys()))==4 and (all(item in list(self.marker_dict.keys()) for item in [1, 5, 13, 17])) and (all(item is not None for item in [self.ur_joint_angle, self.ur_endeffector_position, self.scene_rgb_images, self.aruco_image])):
                traj_data = np.concatenate((np.array([self.gripper_state]),
                        np.array(self.ur_endeffector_position), 
                        np.array(self.ur_joint_angle), 
                        self.marker_dict[1], 
                        self.marker_dict[5],
                        self.marker_dict[13], 
                        self.marker_dict[17]))
                self.record_num += 1
                marker_file_path = self.traj_directory_name + '/marker/' + 'marker_result_' + str(self.record_num) + '.txt'
                traj_file_path = self.traj_directory_name + '/traj/' + 'traj_' + str(self.record_num) + '.npy'
                np.save(traj_file_path, traj_data)
                with open(marker_file_path, 'w') as file:
                    for key, value in self.marker_dict.items():
                        file.write("%s: %s\n" % (key, value))
                cv2.imwrite(self.traj_directory_name +  '/scene_rgb_image/' + 'scene_'  + str(self.record_num) + '.jpg', self.scene_rgb_images)
                cv2.imwrite(self.traj_directory_name +  '/marker_result_image/' + 'aruco_'  + str(self.record_num) + '.jpg', self.aruco_image)
                print("traj", traj_data.shape)
                # print("self.gripper_state ", self.gripper_state)
                # print("self.marker_dict", self.marker_dict)
                # print("self.ur_joint_angle", self.ur_joint_angle)
                # print("self.ur_endeffector_position", self.ur_endeffector_position)

class Auto_Run_Collection:
    def __init__(self):
        rospy.init_node("auto_collect_data")
        self.data = None
        self.is_recording = False
        self.service = rospy.Service('collect_bool_service', SetBool, self.handle_bool_service)
        rospy.loginfo("Service 'collect_bool_service' is ready.")
    def handle_bool_service(self, req):
        if req.data:
            if not self.is_recording:
                self.data = Data_Collection(init_node=False)
                self.is_recording = True
                rospy.loginfo("Started data collection.")
                return SetBoolResponse(success=True, message="Data collection started.")
            else:
                return SetBoolResponse(success=False, message="Already collecting data.")
        else:
            if self.is_recording:
                self.is_recording = False
                rospy.loginfo("Stopped data collection")
                return SetBoolResponse(success=True, message="Data collection stopped.")
            else:
                return SetBoolResponse(success=False, message="Data collection already stopped.")
    def run(self):
        rate = rospy.Rate(4)
        while not rospy.is_shutdown():
            if self.is_recording and self.data is not None:
                self.data.record()
            rate.sleep()

        

if __name__ == "__main__":
    auto_run = Auto_Run_Collection()
    auto_run.run()
        