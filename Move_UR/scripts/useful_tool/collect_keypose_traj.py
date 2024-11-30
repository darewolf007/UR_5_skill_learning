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
    return "[" + ", ".join(map(str, robot_state)) + "],"

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
        rospy.init_node('data_collection')
        self.base_data_path = base_data_path
        self.init_data_dir(base_data_path)
        self.traj_length = 0
        self.traj_end_pose = []
        self.traj_joint_pose = []
        self.traj_rgb_image = []
        self.traj_depth_image = []
        self.ur_endeffector_position = None
        self.gripper = RobotiqGripper(init_node = False)
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

    def record_traj(self, save_image = False):
        traj_save_flag = False
        traj_length = 0
        traj = []
        while not rospy.is_shutdown():
            key = input()
            if key == 'b':
                traj_save_flag = True
                rospy.loginfo("Started saving trajectory")
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
            elif key == 'e':
                traj_save_flag = False 
                rospy.loginfo("Stopped saving trajectory")
            elif key == 'q':
                rospy.loginfo("end saving trajectory")
                break
            elif key == 's':
                rospy.loginfo("not for recording trajectory") 

            if traj_save_flag:
                end_pose = self.ur_endeffector_position
                gripper_state = self.gripper.get_gripper_state()
                if end_pose is not None:
                    robot_state = np.concatenate((end_pose, np.array([gripper_state])))
                    robot_state_str = robot_state_to_string(robot_state)
                    print(str(traj_length) + " : " + robot_state_str)
                    traj_length += 1
                    traj.append(robot_state)
                continue
        numpy_traj = np.array(traj, dtype = np.float32)
        write_npy_file(numpy_traj, self.traj_directory_name + "/traj.npy")

    def select_by_keypoard(self, save_image = False):
        traj = []
        traj_str = []
        traj_length = 0
        while not rospy.is_shutdown():
            key = input()
            if key == 'b':
                rospy.loginfo("not for select keypose") 
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
            elif key == 'e':
                traj_save_flag = False 
                rospy.loginfo("Stopped saving trajectory")
            elif key == 'q':
                rospy.loginfo("end saving trajectory")
                break
            elif key == 's':
                end_pose = self.ur_endeffector_position
                gripper_state = self.gripper.get_gripper_state()
                if end_pose is not None:
                    robot_state = np.concatenate((end_pose, np.array([gripper_state])))
                    robot_state_str = robot_state_to_string(robot_state)
                    traj_length += 1
                    traj.append(robot_state)
                    traj_str.append(robot_state_str)
                    print(str(traj_length) + " : " + robot_state_str)
            else:
                continue
        numpy_traj = np.array(traj, dtype = np.float32)
        write_npy_file(numpy_traj, self.traj_directory_name + "/traj.npy")
        traj_str = "[" + "\n".join(traj_str) + "]"
        with open(self.traj_directory_name + "/traj.txt", "w") as f:
            f.write(traj_str)
      


def main():
    parser = argparse.ArgumentParser(description="Select mode for collecting keypose trajectory")
    parser.add_argument('--mode', type=str, choices=['record', 'select'], required=True, help='Mode to run the script in')
    parser.add_argument('--base_data_path', type=str, default=BASE_DATA_PATH, help='Base data path for saving trajectories')
    args = parser.parse_args()
    collect_data = CollectTrajectory(base_data_path = args.base_data_path)
    if args.mode == 'record':
        collect_data.record_traj()
    elif args.mode == 'select':
        collect_data.select_by_keypoard()
    else:
        raise ValueError("Invalid mode")

if __name__ == "__main__":
    main()

            
