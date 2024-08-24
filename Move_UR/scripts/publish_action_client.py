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
from spirl.real_robot import RealRobot_Test
import torch
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
        # self.color_sub = rospy.Subscriber('/rgb/image_raw', Image, self.collect_scene_rgb_image)
        rospy.wait_for_service('set_robot_action')
        self.set_action_client = rospy.ServiceProxy('/set_robot_action', Action)
        self.robot_move_num = 0
        self.spirl_model = RealRobot_Test()
        self.spirl_model.init_agent()
        self.all_task = [1,2,3,4]
        self.gripper_change_confidence = 0
        with open('/home/yhx/shw/src/Move_UR/scripts/experiments/task_temp_save.txt', 'w') as file:
            file.write(str(self.all_task))
        with open('/home/yhx/shw/src/Move_UR/scripts/experiments/task_temp_save.pkl', 'wb') as file:
            pickle.dump(self.all_task, file)
        
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
                        # np.array([self.gripper_state]),
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
        last_gripper = 0
        gripper_state_change_num = 0
        while not rospy.is_shutdown():
            env_data = self.get_env_state_data()
            if env_data is not None:
                robot_action_data, collect_env_data = self.get_robot_action_data(env_data=env_data, test=False)
                if np.sum(np.abs(np.array(robot_action_data[:-1])))  < 1 and self.robot_gripper_state_check(robot_action_data[-1]) == self.gripper_state:
                    print("not data", robot_action_data)
                    rospy.sleep(0.1)
                    continue
                #####!!!!!!! change robot action *1000/100
                if robot_action_data is None:
                        break
                # action = self.spirl_model.ruself.robot_gripper_state_check(robot_action_data[-1]) == self.gripper_state:n_once(collect_env_data)
                # print("my action", action/1000)
                # print("robot_action_data", robot_action_data)
                if robot_action_data.shape[0] == 7:
                    print(robot_action_data)
                    # print("env_data", env_data[:6])
                    robot_action_data[:-1] = robot_action_data[:-1]/100 + env_data[:6]
                    # print("action", robot_action_data)
                    # print("robot_action_data", robot_action_data)
                else:
                    if self.ur_endeffector_position is not None:
                        print(robot_action_data)
                        robot_action_data[:-1] = robot_action_data[:-1]/1000 + self.ur_endeffector_position[:7]
                        # print("action", robot_action_data)
                        # print("robot_action_data", robot_action_data)
                        if self.robot_gripper_state_check(robot_action_data[-1]) != self.gripper_state:
                            self.gripper_change_confidence += 1
                        if self.gripper_change_confidence > 3:
                            robot_action_data[-1] = self.robot_gripper_state_check(robot_action_data[-1])
                        else:
                            robot_action_data[-1] = self.gripper_state
                        # robot_action_data[-1] = self.gripper_state
                robot_state = self.send_robot_action(robot_action_data)
                self.robot_move_num += 1
                print(robot_state)
                if robot_state:
                    print('hereeeeee')
                    now_gripper = robot_action_data[-1]
                    print("now_gripper", now_gripper)
                    if now_gripper != last_gripper:
                        gripper_state_change_num += 1
                        self.gripper_change_confidence = 0
                        # if now_gripper == 0 and last_gripper ==1:
                        #     print("one task is done")
                        #     self.all_task.pop(0)
                        #     print(self.all_task)
                        #     with open('/home/yhx/shw/src/Move_UR/scripts/experiments/task_temp_save.pkl', 'wb') as file:
                        #         pickle.dump(self.all_task, file)
                        last_gripper = now_gripper
                        print("gripper state", self.gripper_state)
                else:
                    break
                # if gripper_state_change_num == 8:
                #     rospy.loginfo("all task is done")
                #     break
if __name__ == "__main__":
    move_client = MoveClient()
    move_client.run()