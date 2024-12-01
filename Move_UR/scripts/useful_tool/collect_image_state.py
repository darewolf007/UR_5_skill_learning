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
from kinect_camera import KinectDK
from scipy import misc
from sensor_msgs.msg import JointState
BASE_DATA_PATH = "/home/yhx/shw/src/Dataset_Collection/keypose/"

def write_npy_file(data, file_name):
    np.save(file_name, data)

def robot_state_to_string(robot_state):
    return "[" + ", ".join(map(str, robot_state)) + "]"

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
        self.ur_joint_angle = None
        self.ur_joint_velocity=None
        self.gripper = RobotiqGripper(init_node = True)
        self.ur_endeffector_sub = rospy.Subscriber('/tf', TFMessage, self.collect_UR_endeffector_position)
        self.ur_joint_sub = rospy.Subscriber('/joint_states', JointState, self.collect_UR_joint_info)

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
    
    def collect_UR_joint_info(self, joint_state):
        self.ur_joint_angle = np.array(joint_state.position)
        self.ur_joint_velocity=np.array(joint_state.velocity)
    
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
        save_path =dataset_path+'/rgb/rgb_'+str(count)+'.png'
        cv2.imwrite(save_path, img_color)
        save_path2 = dataset_path+ '/depth/depth_'+str(count)+'.png'
        save_path3 = dataset_path+ '/depth/depth_'+str(count)+'.npy'
        misc.imsave(save_path2, img_depth)
        np.save(save_path3, img_depth.astype(np.float16))
    
    def save_step_state(self,count):
        end_pose = self.ur_endeffector_position
        gripper_state = self.gripper.get_gripper_state()
        joint_state = self.ur_joint_angle
        if end_pose is not None:
            robot_state = np.concatenate((end_pose, np.array([gripper_state])))
            robot_joint_state = np.concatenate((joint_state, np.array([gripper_state])))
            robot_state_str = robot_state_to_string(robot_state)
            robot_joint_str = robot_state_to_string(robot_joint_state)
            numpy_traj = np.array(robot_state, dtype = np.float32)
            numpy_joint = np.array(robot_joint_state, dtype = np.float32)
            write_npy_file(numpy_joint, self.traj_directory_name + '/state/joint_trajectory_'+str(count)+'.npy')
            write_npy_file(numpy_traj, self.traj_directory_name + '/state/end_trajectory_'+str(count)+'.npy')
            with open(self.traj_directory_name + '/state/end_trajectory_'+str(count)+'.txt', "w") as f:
                f.write(robot_state_str)
            with open(self.traj_directory_name + '/state/joint_trajectory_'+str(count)+'.txt', "w") as f:
                f.write(robot_joint_str)

    def get_keyboard_image_state(self):
        kinect_dk = KinectDK()
        img_color = kinect_dk.queue_color.get(timeout=10.0)
        K = np.asarray(kinect_dk.rgb_info.K).reshape(3, 3)
        D = np.asarray(kinect_dk.rgb_info.D)
        size = img_color.shape[:2][::-1]
        map1, map2 = cv2.initUndistortRectifyMap(K, D, None, None, size, cv2.CV_32FC1)
        count = 1
        while not rospy.is_shutdown():
            ori_img_color = kinect_dk.queue_color.get(timeout=10.0)
            ori_img_depth = kinect_dk.queue_depth.get(timeout=10.0)
            img_color = cv2.remap(ori_img_color, map1, map2, cv2.INTER_CUBIC)
            img_depth = cv2.remap(ori_img_depth, map1, map2, cv2.INTER_NEAREST)
            img_color_disp = cv2.resize(img_color, tuple(np.asarray(size)//2))
            img_depth_disp = cv2.resize(img_depth, tuple(np.asarray(size)//2))
            cv2.imshow('img', img_color_disp)
            cv2.imshow('dpth',img_depth_disp )
            key = cv2.waitKey(1)
            if key == ord('s'):
                self.save_step_image(count, ori_img_color, ori_img_depth)
                self.save_step_state(count)
                print('success save',count)
                count+=1
            elif key ==ord( 'a'):
                self.gripper.activate_gripper()
                rospy.loginfo("Gripper activated")
            elif key ==ord( 'r'):
                self.gripper.reset_gripper()
                rospy.loginfo("Gripper reset")
            elif key ==ord( 'o'):
                self.gripper.open_gripper()
                rospy.loginfo("Gripper opened")
            elif key ==ord( 'c'):
                self.gripper.close_gripper()    
                rospy.loginfo("Gripper closed") 
            elif key == ord('q'):
                cv2.destroyAllWindows()
                break
        kinect_dk.release()

def main():
    parser = argparse.ArgumentParser(description="Select mode for collecting keypose trajectory")
    parser.add_argument('--base_data_path', type=str, default=BASE_DATA_PATH, help='Base data path for saving trajectories')
    args = parser.parse_args()
    collect_data = CollectTrajectory(base_data_path = args.base_data_path)
    collect_data.get_keyboard_image_state()

if __name__ == "__main__":
    main()

            
