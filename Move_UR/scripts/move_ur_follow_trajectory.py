#!/usr/bin/env python2
from __future__ import print_function
import sys
import rospy
import os
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
import rospy
import numpy as np
from std_srvs.srv import SetBool, SetBoolRequest
from utils.ur_trajectory import TrajectoryClient

class ShwUr5eMoveClient:
    def __init__(self, gripper):
        self.auto_collect_state = False
        self.gripper = gripper
        self.client = TrajectoryClient(init_node = False)
        
    def call_auto_record_service(self, flag):
        rospy.wait_for_service("collect_bool_service")
        try:
            service_proxy = rospy.ServiceProxy("collect_bool_service", SetBool)
            request = SetBoolRequest(data=flag)
            response = service_proxy(request)
            self.auto_collect_state = response
        except rospy.ServiceException as e:
            request = SetBoolRequest(data=flag)
    
    def joint_traj_move(self, file = None, is_collect = False):
        if file is not None:
            joint_pos = np.load(file)
            if is_collect:
                self.client.move_once_by_end(joint_pos[0][:6])
                if joint_pos[0][6]:
                    self.gripper.close_gripper()
                else:
                    self.gripper.open_gripper()
                self.call_auto_record_service(True)
                rospy.sleep(0.1)
            for i in range(joint_pos.shape[0]):
                self.client.move_once_by_joint(joint_pos[i][:6], need_time = 1)
                rospy.sleep(0.05)
                if joint_pos[i][6]:
                    self.gripper.close_gripper()
                else:
                    self.gripper.open_gripper()
            if is_collect:
                self.call_auto_record_service(False)
        
    def end_traj_move(self, file = None, is_collect = False, add_random = False):
        if file is not None:
            end_pos = np.load(file)
            if is_collect:
                self.client.move_once_by_end(end_pos[0][:7], need_time = 2.5)
                if end_pos[0][7]:
                    self.gripper.close_gripper()
                else:
                    self.gripper.open_gripper()
                self.call_auto_record_service(True)
            for i in range(end_pos.shape[0]):
                self.client.move_once_by_end(end_pos[i][:7], need_time = 1)
                if end_pos[i][7]:
                    self.gripper.close_gripper()
                else:
                    self.gripper.open_gripper()
            if is_collect:
                self.call_auto_record_service(False)

# def step_npy_files_in_order(directory):
#     rospy.init_node("move_traj_once")
#     files = [f for f in os.listdir(directory) if f.endswith('.npy')]
#     client = TrajectoryClient(init_node = False)
#     client.send_init_joint_trajectory()
#     gripper = RobotiqGripper(init_node = False)
#     files.sort(key=lambda x: int(os.path.splitext(x)[0].split("_")[1]))
#     last_gripper = 0
#     now_gripper = 1
#     for file in files:
#         filepath = os.path.join(directory, file)
#         data = np.load(filepath)
#         now_gripper = data[0]
#         ur_endeffector_position = data[1:8]
#         ur_joint_angle = data[8:14]
#         print("here!!!")
#         print("ur_endeffector_position", ur_endeffector_position)
#         print("ur_joint_angle", ur_joint_angle)
#         # client.move_once_by_end(ur_endeffector_position)
#         client.move_once_by_joint(ur_joint_angle)
#         print("now_gripper", now_gripper)
#         print("last_gripper", last_gripper)
#         if now_gripper != last_gripper:
#             if now_gripper:
#                 gripper.close_gripper()
#             else:
#                 gripper.open_gripper()
#             last_gripper = now_gripper
#     return data
        
if __name__ == "__main__":
    # test_path = "/home/yhx/shw/src/Dataset_Collection/ABCD_all_example/traj"
    # step_npy_files_in_order("/home/yhx/shw/src/Dataset_Collection/ABCD_all_example/traj")
    rospy.init_node("move_traj_once")
    is_collect = rospy.get_param('~is_collect')
    control_mode = rospy.get_param('~control_mode')
    trajectory_path = rospy.get_param('~trajectory_path')
    local_gripper_control = rospy.get_param('~local_gripper_communicate')
    robot_ip = rospy.get_param('~robot_ip')
    if local_gripper_control:
        from utils.robotiq_gripper import RobotiqGripper
        gripper = RobotiqGripper(init_node = False)
    else:
        from utils.robotiq_gripper_remote import RobotiqGripper
        gripper = RobotiqGripper(robot_ip)
    run_traj = ShwUr5eMoveClient(gripper)
    if control_mode == "end":
        run_traj.end_traj_move(trajectory_path, is_collect)
    elif control_mode == "joint":
        run_traj.joint_traj_move(trajectory_path, is_collect)
    else:
        print("control_mode is not correct")