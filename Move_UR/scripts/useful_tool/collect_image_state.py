#!/usr/bin/env python2
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
import rospy
import re
import subprocess
import cv2
import rospy
import numpy as np
import os
import argparse
from control_robotiq import RobotiqGripper
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_input
from tf2_msgs.msg import TFMessage
from utils.kinect_camera import KinectDK
from scipy import misc
BASE_DATA_PATH = "/home/yhx/shw/src/Dataset_Collection/keypose/"

def write_npy_file(data, file_name):
    np.save(file_name, data)

def read_end_pose():
    output = subprocess.Popen(["rostopic", "echo", "-n", "1", "/tf"], stdout=subprocess.PIPE).stdout.read().decode()
    output = output.strip().split("\n")
    x = output[11].strip().split()[1]
    y = output[12].strip().split()[1]
    z = output[13].strip().split()[1]
    rx = output[15].strip().split()[1]
    ry = output[16].strip().split()[1]
    rz = output[17].strip().split()[1]
    rw = output[18].strip().split()[1]
    return "[" + ", ".join([x, y, z, rx, ry, rz, rw]) + "],", [x, y, z, rx, ry, rz, rw]

def robot_state_to_string(robot_state):
    return "[" + ", ".join(map(str, robot_state)) + "]"

class RobotiqGripper:
    def __init__(self, init_node = True, commend_control = False):
        if init_node:
            rospy.init_node('Robotiq2FGripperSimpleController')
        self.gripper_pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)
        self.gripper_command = outputMsg.Robotiq2FGripper_robot_output()
        self.init_gripper()
        self.gripper_state = 0
        self.ur_gripper_sub = rospy.Subscriber('/Robotiq2FGripperRobotInput', Robotiq2FGripper_robot_input, self._collect_gripper_state)

    def init_gripper(self):
        self.reset_gripper()
        self.activate_gripper()
        self.open_gripper()
        
    def reset_gripper(self):
        self.gripper_command = outputMsg.Robotiq2FGripper_robot_output()
        self.gripper_command.rACT = 0
        self.gripper_pub.publish(self.gripper_command)
    
    def activate_gripper(self):
        self.gripper_command = outputMsg.Robotiq2FGripper_robot_output()
        self.gripper_command.rACT = 1
        self.gripper_command.rGTO = 1
        self.gripper_command.rSP  = 255
        self.gripper_command.rFR  = 150
        self.gripper_pub.publish(self.gripper_command)
        
    def open_gripper(self):
        self.gripper_command.rPR = 0  
        self.gripper_pub.publish(self.gripper_command)
    
    def close_gripper(self):
        self.gripper_command.rPR = 255
        self.gripper_pub.publish(self.gripper_command)

    def _collect_gripper_state(self, command):
        if command.gPR == 0:
            self.gripper_state = 0
        else:
            self.gripper_state = 1
    
    def get_gripper_state(self):
        return self.gripper_state

class CollectTrajectory:
    def __init__(self, base_data_path = BASE_DATA_PATH):
        self.base_data_path = base_data_path
        self.init_data_dir(base_data_path)
        self.traj_length = 0
        self.traj_end_pose = []
        self.traj_joint_pose = []
        self.traj_rgb_image = []
        self.traj_depth_image = []
        self.ur_endeffector_position = None
        self.gripper = RobotiqGripper(init_node = True)
        self.ur_endeffector_sub = rospy.Subscriber('/tf', TFMessage, self.collect_UR_endeffector_position)
    
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
    
    def count_dirs_in_directory(self, path):
        count = sum(os.path.isdir(os.path.join(path, name)) for name in os.listdir(path))
        return count

    def init_data_dir(self, base_data_path):
        self.traj_directory_name = base_data_path + str(self.count_dirs_in_directory(base_data_path) +1)
        os.mkdir(self.traj_directory_name)
        os.mkdir(self.traj_directory_name + "/rgb")
        os.mkdir(self.traj_directory_name + "/depth")
        os.mkdir(self.traj_directory_name + "/state")
    
    def save_step_image(self, count, img_color, img_depth):
        dataset_path = self.traj_directory_name
        save_path = os.path.join(dataset_path, 'rgb/rgb_'+str(count)+'.jpg')
        cv2.imwrite(save_path, img_color)
        save_path2 = os.path.join(dataset_path, 'depth/depth_'+str(count)+'.png')
        cv2.imwrite(save_path2, np.uint16(img_depth))
    
    def save_step_state(self,count):
        end_pose = self.ur_endeffector_position
        gripper_state = self.gripper.get_gripper_state()
        if end_pose is not None:
            robot_state = np.concatenate((end_pose, np.array([gripper_state])))
            robot_state_str = robot_state_to_string(robot_state)
            traj_length += 1
            print(str(traj_length) + " : " + robot_state_str)
            numpy_traj = np.array(robot_state, dtype = np.float32)
            write_npy_file(numpy_traj, self.traj_directory_name + 'traj/traj_'+str(count)+'.npy')
            with open(self.traj_directory_name + "/traj.txt", "w") as f:
                f.write(robot_state_str)

    def get_keyboard_image_state(self):
        kinect_dk = KinectDK()
        img_color = kinect_dk.queue_color.get(timeout=10.0)
        K = np.asarray(kinect_dk.rgb_info.K).reshape(3, 3)
        D = np.asarray(kinect_dk.rgb_info.D)
        size = img_color.shape[:2][::-1]
        map1, map2 = cv2.initUndistortRectifyMap(K, D, None, None, size, cv2.CV_32FC1)
        count = 1
        while not rospy.is_shutdown():
            img_color = kinect_dk.queue_color.get(timeout=10.0)
            img_depth = kinect_dk.queue_depth.get(timeout=10.0)
            img_color = cv2.remap(img_color, map1, map2, cv2.INTER_CUBIC)
            img_depth = cv2.remap(img_depth, map1, map2, cv2.INTER_NEAREST)
            img_color_disp = cv2.resize(img_color, tuple(np.asarray(size)//2))
            img_depth_disp = cv2.resize(img_depth, tuple(np.asarray(size)//2))
            cv2.imshow('img', img_color_disp)
            cv2.imshow('dpth',img_depth_disp )
            key = cv2.waitKey(1)
            if key == ord('s'):
                self.save_step_image(count, img_color, img_depth)
                self.save_step_state(count)
                print('success save',count)
                count+=1
            elif key == 'a':
                self.gripper.activate_gripper()
                rospy.loginfo("Gripper activated")
            elif key == 'r':
                self.gripper.reset_gripper()
                rospy.loginfo("Gripper reset")
            elif key == 'o':
                self.gripper.open_gripper()
                rospy.loginfo("Gripper opened")
            elif key == 'c':
                self.gripper.close_gripper()    
                rospy.loginfo("Gripper closed") 
            elif key == ord('q'):
                cv2.destroyAllWindows()
                break
        kinect_dk.release()

def main():
    collect_data = CollectTrajectory(base_data_path = args.base_data_path)
    collect_data.get_keyboard_image_state()

if __name__ == "__main__":
    main()

            