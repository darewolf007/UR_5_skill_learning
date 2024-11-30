import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
import rospy
import re
import subprocess
import cv2
import rospy
import numpy as np
from control_robotiq import RobotiqGripper
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg

class RobotiqGripper:
    def __init__(self, init_node = True, commend_control = False):
        if init_node:
            rospy.init_node('Robotiq2FGripperSimpleController')
        self.gripper_pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)
        self.gripper_command = outputMsg.Robotiq2FGripper_robot_output()
        self.init_gripper()
        if commend_control:
            while not rospy.is_shutdown():
                self.gripper_command = self.gen_gripper_command(self.askForCommand(self.gripper_command), self.gripper_command)
                self.gripper_pub.publish(self.gripper_command) 
                rospy.sleep(0.1)
        
        
    def init_gripper(self):
        self.reset_gripper()
        self.activate_gripper()
        self.open_gripper()
        
    def reset_gripper(self):
        self.gripper_command = outputMsg.Robotiq2FGripper_robot_output()
        self.gripper_command.rACT = 0
        self.gripper_pub.publish(self.gripper_command)
        rospy.sleep(0.1)
    
    def activate_gripper(self):
        self.gripper_command = outputMsg.Robotiq2FGripper_robot_output()
        self.gripper_command.rACT = 1
        self.gripper_command.rGTO = 1
        self.gripper_command.rSP  = 255
        self.gripper_command.rFR  = 150
        self.gripper_pub.publish(self.gripper_command)
        rospy.sleep(0.1)
        
    def open_gripper(self):
        self.gripper_command.rPR = 0  
        self.gripper_pub.publish(self.gripper_command)
        rospy.sleep(0.1)
    
    def close_gripper(self):
        self.gripper_command.rPR = 255
        self.gripper_pub.publish(self.gripper_command)
        rospy.sleep(0.1)


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
    return "[" + ", ".join([x, y, z, rx, ry, rz, rw]) + "],"

def record_traj(save_image = False):
    traj_save_flag = False
    gripper = RobotiqGripper(init_node = True)
    while not rospy.is_shutdown():
        key = input()
        if key == 'b':
            traj_save_flag = True
        elif key == 'o':
            gripper.open_gripper()
        elif key == 'c':
            gripper.close_gripper()     
        elif key == 'e':
            traj_save_flag = False 
        else:
            continue
        if traj_save_flag:
            pose = read_end_pose()

def select_by_keypoard(save_image = False):
    traj_length = 0
    traj_end_pose = []
    traj_joint_pose = []
    traj_rgb_image = []
    traj_depth_image = []
    gripper = RobotiqGripper(init_node = True)
    while not rospy.is_shutdown():
        key = input()
        if key == 'e':
            break
        elif key == 'o':
            gripper.open_gripper()
        elif key == 'c':
            gripper.close_gripper()       
        elif key == 'q':
            break
        else:
            continue
        pose = read_end_pose()
        traj_length += 1
        print(traj_length, pose)
        traj_end_pose.append(pose)
    traj_str = "[" + "\n".join(traj_end_pose) + "]"
    with open("traj.txt", "w") as f:
        f.write(traj_str)
    
    
def run():
    traj = []
    gripper = RobotiqGripper(init_node = False)
    while not rospy.is_shutdown():
        key = cv2.waitKey(1)
        if key == ord('e'):
            break
        elif key == ord('o'):
            gripper.open_gripper()
        elif key == ord('c'):
            gripper.close_gripper()       
        elif key == ord('q'):
            break
        else:
            continue
        pose = read_end_pose()
        traj_length += 1
        print(traj_length, pose)
        traj.append(pose)
    traj_str = "[" + "\n".join(traj) + "]"
    with open("traj.txt", "w") as f:
        f.write(traj_str)
        
if __name__ == "__main__":
    select_by_keypoard()

            
