import sys
import os
import rospy
import numpy as np
import cv2
import threading
import pickle
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, JointState
from aruco_msgs.msg import MarkerArray
from tf2_msgs.msg import TFMessage
from Move_UR.srv import Action, ActionResponse
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_input
import torch
import requests
import io
from utils.kinect_camera import KinectDK

SERVER_URL = "http://10.184.17.105:8000/act"
instr = "pickup object 20cm"
 

def send_data(image_array: np.ndarray, depth_array: np.ndarray, proprio_array,  timestep):
    # 将图像数组转换为字节流
    _, image_encoded = cv2.imencode('.jpg', image_array)
    image_bytes = io.BytesIO(image_encoded.tobytes())

    # 将深度数组转换为字节流（使用 .npy 格式）
    depth_bytes = io.BytesIO()
    np.save(depth_bytes, depth_array)
    depth_bytes.seek(0)
    
    proprio_bytes = io.BytesIO()
    np.save(proprio_bytes, proprio_array)
    proprio_bytes.seek(0)

    # 定义要发送的数据
    files = {
        "image_file": ("image.jpg", image_bytes, "image/jpeg"),
        "depth_file": ("depth.npy", depth_bytes, "application/octet-stream"),
        "proprio_file": ("proprio.npy", proprio_bytes, "application/octet-stream"),
    }
    
    data = {
        "instr": instr,
        "timestep": timestep
    }

    # 发送 POST 请求到服务端
    response = requests.post(SERVER_URL, files=files, data=data)

    # 打印服务端返回的结果
    if response.status_code == 200:
        print("Server response:", response.json())
        return response.json()
    else:
        print("Failed to get a valid response from server. Status code:", response.status_code)
        return None

def read_pickle(filename):
    with open(filename, 'rb') as f:
        loaded_data = pickle.load(f, encoding='latin1')
    return loaded_data

class MoveClient:
    def __init__(self):
        rospy.init_node('action_client')
        self.marker_dict = {}
        self.ur_joint_angle = None
        self.scene_rgb_images = None
        self.ur_endeffector_position = None
        self.gripper_state = 0
        self.last_env = None
        self.cvbridge = CvBridge()
        self.robot_initial_joint = [1.542957607899801, -1.3407166761210938, 1.2018647193908691, -1.75225891689443, -1.6117513815509241, -0.3191445509540003]
        self.safe_lock = threading.Lock()
        self.aruco_result_sub = rospy.Subscriber('/aruco_marker_publisher/markers', MarkerArray, self.collect_aruco_results)
        self.ur_joint_sub = rospy.Subscriber('/joint_states', JointState, self.collect_UR_joint_angle)
        self.ur_gripper_sub = rospy.Subscriber('/Robotiq2FGripperRobotInput', Robotiq2FGripper_robot_input, self.collect_gripper_state)
        self.ur_endeffector_sub = rospy.Subscriber('/tf', TFMessage, self.collect_UR_endeffector_position)
        rospy.wait_for_service('set_robot_action')
        self.set_action_client = rospy.ServiceProxy('/set_robot_action', Action)
        self.robot_move_num = 0
        self.init_camera()
        
    def init_camera(self):
        self.kinect_dk = KinectDK()
        img_color = self.kinect_dk.queue_color.get(timeout=10.0)
        
        K = np.asarray(self.kinect_dk.rgb_info.K).reshape(3, 3)
        D = np.asarray(self.kinect_dk.rgb_info.D)
        self.size = img_color.shape[:2][::-1]
        self.map1, self.map2 = cv2.initUndistortRectifyMap(K, D, None, None, self.size, cv2.CV_32FC1)
        
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
        
    def collect_gripper_state(self, command):
        if command.gPR == 0:
            self.gripper_state = 0
        else:
            self.gripper_state = 1
        # print("gripper state", self.gripper_state)
    
    def collect_scene_rgb_image(self, image_msg):
        image = self.cvbridge.imgmsg_to_cv2(image_msg,  desired_encoding='rgb8')
        self.scene_rgb_images = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)

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
    
    def get_test_data(self, file, test_joint = True):
        data = read_pickle(file)
        print("self.robot_move_num", self.robot_move_num)
        if test_joint:
            robot_action = data['joint_actions'][self.robot_move_num]
        else:
            robot_action = data['endeffector_actions'][self.robot_move_num]
        if self.robot_move_num == np.array(data['env_state'][1:]).shape[0]:
            return None
        sub_task_id = np.zeros(np.array(data['env_state'][1:]).shape[0], dtype=np.int32)
        for key, value in data['subtask_info'].items():
            subtask_id = value['done_task']
            subtask_begin_idx = value['begin_task_idx']
            subtask_end_idx = value['done_task_idx']
            sub_task_id[subtask_begin_idx:subtask_end_idx] = subtask_id
        print("task id: ", sub_task_id[self.robot_move_num])
        env_data = data['env_state'][self.robot_move_num]
        return robot_action, env_data

    def send_robot_action(self, action):
        try:
            robot_resp = self.set_action_client(action)
            return robot_resp.success
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            return False
    
    def init_robot_pose(self, action):
        try:
            robot_resp = self.set_action_client(action)
            return robot_resp.success
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            return False
    
    def robot_gripper_state_check(self, data):
        # data = np.tanh(data)
        if data > 0:
            return 1
        else:
            return 0
    
    def get_env_state_data(self, use_image = False):
        if use_image:
            if (all(item in list(self.marker_dict.keys()) for item in [1, 5, 13, 17])) and (all(item is not None for item in [self.ur_joint_angle, self.scene_rgb_images])):
                pass
        else:
            with self.safe_lock:         
                if (all(item in list(self.marker_dict.keys()) for item in [1, 5, 13, 17])) and (all(item is not None for item in [self.ur_joint_angle])):
                    env_data = np.concatenate((np.array(self.ur_joint_angle), 
                        np.array([self.gripper_state]),
                        self.marker_dict[1], 
                        self.marker_dict[5],
                        self.marker_dict[13], 
                        self.marker_dict[17]))
                    self.last_env = env_data
                    return env_data
                else:
                    print("env wrong")
                    if self.last_env is not None:
                        return self.last_env
                    return None
    
    def get_robot_action_data(self, env_data = None, test = False):
        if test:
            action_data, env_data = self.get_test_data("/home/yhx/shw/src/Dataset_Collection/sample_abcd_1_5.pkl", test_joint=False)
        else:
            # test_action_data, env_data = self.get_test_data("/home/yhx/shw/src/Dataset_Collection/sample_abcd_1_5.pkl", test_joint=False)
            action_data = self.spirl_model.run_once(env_data)
        return action_data, env_data
    
    def run(self):
        timestep = 0
        while not rospy.is_shutdown():
            img_color = self.kinect_dk.queue_color.get(timeout=10.0)
            img_depth = self.kinect_dk.queue_depth.get(timeout=10.0)
            proprio_array = np.array([self.gripper_state])
            proprio_array = np.concatenate([self.ur_endeffector_position, proprio_array])
            if img_color is not None:
                action_dict = send_data(img_color, img_depth, proprio_array, timestep)
                robot_action_data = action_dict["action"]
                print(robot_action_data)
                rospy.sleep(1)
                robot_state = self.send_robot_action(np.array(robot_action_data))
                timestep += 1
                
                

                
if __name__ == "__main__":
    move_client = MoveClient()
    move_client.run()