#!/usr/bin/env python2
 
from __future__ import print_function

import sys
import rospy
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
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolRequest
# Compatibility for python2 and python3
if sys.version_info[0] < 3:
    input = raw_input

# If your robot description is created with a tf_prefix, those would have to be adapted
JOINT_NAMES = [
    "elbow_joint",
    "shoulder_lift_joint",
    "shoulder_pan_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# All of those controllers can be used to execute joint-based trajectories.
# The scaled versions should be preferred over the non-scaled versions.
JOINT_TRAJECTORY_CONTROLLERS = [
    "scaled_pos_joint_traj_controller",
    "scaled_vel_joint_traj_controller",
    "pos_joint_traj_controller",
    "vel_joint_traj_controller",
    "forward_joint_traj_controller",
]

# All of those controllers can be used to execute Cartesian trajectories.
# The scaled versions should be preferred over the non-scaled versions.
CARTESIAN_TRAJECTORY_CONTROLLERS = [
    "pose_based_cartesian_traj_controller",
    "joint_based_cartesian_traj_controller",
    "forward_cartesian_traj_controller",
]

# We'll have to make sure that none of these controllers are running, as they will
# be conflicting with the joint trajectory controllers
CONFLICTING_CONTROLLERS = ["joint_group_vel_controller", "twist_controller"]

class RobotiqGripper:
    def __init__(self, init_node = True, commend_control = False):
        if init_node:
            rospy.init_node('Robotiq2FGripperSimpleController')
        self.gripper_pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)
        self.gripper_state_pub = rospy.Publisher('ur_gripper', Bool)
        self.gripper_command = outputMsg.Robotiq2FGripper_robot_output()
        self.gripper_state = False
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
        self.gripper_state = False
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
        gripper_msg = Bool()
        gripper_msg.data = False
        self.gripper_state_pub.publish(gripper_msg)
        rospy.sleep(0.1)
    
    def close_gripper(self):
        self.gripper_command.rPR = 255
        self.gripper_pub.publish(self.gripper_command)
        gripper_msg = Bool()
        gripper_msg.data = True
        self.gripper_state_pub.publish(gripper_msg)
        rospy.sleep(0.1)
    
    def gen_gripper_command(self, char, command):
        """Update the command according to the character entered by the user."""    
        
        if char == 'a':
            self.activate_gripper()

        if char == 'r':
            self.reset_gripper()

        if char == 'c':
            self.close_gripper()

        if char == 'o':
            self.open_gripper()

        #If the command entered is a int, assign this value to rPRA
        try: 
            command.rPR = int(char)
            if command.rPR > 255:
                command.rPR = 255
            if command.rPR < 0:
                command.rPR = 0
        except ValueError:
            pass                    
            
        if char == 'f':
            command.rSP += 25
            if command.rSP > 255:
                command.rSP = 255
                
        if char == 'l':
            command.rSP -= 25
            if command.rSP < 0:
                command.rSP = 0

                
        if char == 'i':
            command.rFR += 25
            if command.rFR > 255:
                command.rFR = 255
                
        if char == 'd':
            command.rFR -= 25
            if command.rFR < 0:
                command.rFR = 0

        return command
    
    def askForCommand(self, command):
        """Ask the user for a command to send to the gripper."""    

        currentCommand  = 'Simple 2F Gripper Controller\n-----\nCurrent command:'
        currentCommand += '  rACT = '  + str(command.rACT)
        currentCommand += ', rGTO = '  + str(command.rGTO)
        currentCommand += ', rATR = '  + str(command.rATR)
        currentCommand += ', rPR = '   + str(command.rPR )
        currentCommand += ', rSP = '   + str(command.rSP )
        currentCommand += ', rFR = '   + str(command.rFR )


        print(currentCommand)

        strAskForCommand  = '-----\nAvailable commands\n\n'
        strAskForCommand += 'r: Reset\n'
        strAskForCommand += 'a: Activate\n'
        strAskForCommand += 'c: Close\n'
        strAskForCommand += 'o: Open\n'
        strAskForCommand += '(0-255): Go to that position\n'
        strAskForCommand += 'f: Faster\n'
        strAskForCommand += 'l: Slower\n'
        strAskForCommand += 'i: Increase force\n'
        strAskForCommand += 'd: Decrease force\n'
        
        strAskForCommand += '-->'
        # print(raw_input(strAskForCommand))

        return raw_input(strAskForCommand)

class TrajectoryClient:
    """Small trajectory client to test a joint trajectory"""

    def __init__(self, init_node = True):
        if init_node:
            rospy.init_node("my_reset_move")

        timeout = rospy.Duration(5)
        self.switch_srv = rospy.ServiceProxy(
            "controller_manager/switch_controller", SwitchController
        )
        self.load_srv = rospy.ServiceProxy("controller_manager/load_controller", LoadController)
        self.list_srv = rospy.ServiceProxy("controller_manager/list_controllers", ListControllers)
        try:
            self.switch_srv.wait_for_service(timeout.to_sec())
        except rospy.exceptions.ROSException as err:
            rospy.logerr("Could not reach controller switch service. Msg: {}".format(err))
            sys.exit(-1)

        self.joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[0]
        self.cartesian_trajectory_controller = CARTESIAN_TRAJECTORY_CONTROLLERS[0]

    def send_init_joint_trajectory(self, joint_pos = [1.542957607899801, -1.3407166761210938, 1.2018647193908691, -1.75225891689443, -1.6117513815509241, -0.3191445509540003], need_time = 1.0):
        """Creates a trajectory and sends it using the selected action server"""

        # make sure the correct controller is loaded and activated
        self.switch_controller(self.joint_trajectory_controller)
        trajectory_client = actionlib.SimpleActionClient(
            "{}/follow_joint_trajectory".format(self.joint_trajectory_controller),
            FollowJointTrajectoryAction,
        )

        # Wait for action server to be ready
        timeout = rospy.Duration(5)
        if not trajectory_client.wait_for_server(timeout):
            rospy.logerr("Could not reach controller action server.")
            sys.exit(-1)

        # Create and fill trajectory goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = JOINT_NAMES

        # The following list are arbitrary positions
        # Change to your own needs if desired
        position_list = [joint_pos]
        duration_list = [need_time]
        for i, position in enumerate(position_list):
            point = JointTrajectoryPoint()
            point.positions = position
            point.time_from_start = rospy.Duration(duration_list[i])
            goal.trajectory.points.append(point)

        # self.ask_confirmation(position_list)
        rospy.loginfo("Executing trajectory using the {}".format(self.joint_trajectory_controller))

        trajectory_client.send_goal(goal)
        trajectory_client.wait_for_result()

        result = trajectory_client.get_result()
        rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))

    def send_init_cartesian_trajectory(self, end_pos = [0.249179509371,-0.392681547898, 0.38588219159, 0.927532761465,0.368790596434,0.0409693418325,0.044698830199], need_time = 1.0):
        """Creates a Cartesian trajectory and sends it using the selected action server"""
        self.switch_controller(self.cartesian_trajectory_controller)

        # make sure the correct controller is loaded and activated
        goal = FollowCartesianTrajectoryGoal()
        trajectory_client = actionlib.SimpleActionClient(
            "{}/follow_cartesian_trajectory".format(self.cartesian_trajectory_controller),
            FollowCartesianTrajectoryAction,
        )

        # Wait for action server to be ready
        timeout = rospy.Duration(5)
        if not trajectory_client.wait_for_server(timeout):
            rospy.logerr("Could not reach controller action server.")
            sys.exit(-1)

        pose_list = [
            geometry_msgs.Pose(
                geometry_msgs.Vector3(end_pos[:3]), geometry_msgs.Quaternion(end_pos[3:]) 
            ),
        ]
        for i, pose in enumerate(pose_list):
            point = CartesianTrajectoryPoint()
            point.pose = pose
            point.time_from_start = need_time
            goal.trajectory.points.append(point)

        # self.ask_confirmation(pose_list)
        rospy.loginfo(
            "Executing trajectory using the {}".format(self.cartesian_trajectory_controller)
        )
        trajectory_client.send_goal(goal)
        trajectory_client.wait_for_result()

        result = trajectory_client.get_result()

        rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))

    def ask_confirmation(self, waypoint_list):
        """Ask the user for confirmation. This function is obviously not necessary, but makes sense
        in a testing script when you know nothing about the user's setup."""
        rospy.logwarn("The robot will move to the following waypoints: \n{}".format(waypoint_list))
        confirmed = False
        valid = False
        while not valid:
            input_str = input(
                "Please confirm that the robot path is clear of obstacles.\n"
                "Keep the EM-Stop available at all times. You are executing\n"
                "the motion at your own risk. Please type 'y' to proceed or 'n' to abort: "
            )
            valid = input_str in ["y", "n"]
            if not valid:
                rospy.loginfo("Please confirm by entering 'y' or abort by entering 'n'")
            else:
                confirmed = input_str == "y"
        if not confirmed:
            rospy.loginfo("Exiting as requested by user.")
            sys.exit(0)

    def choose_controller(self):
        """Ask the user to select the desired controller from the available list."""
        rospy.loginfo("Available trajectory controllers:")
        for (index, name) in enumerate(JOINT_TRAJECTORY_CONTROLLERS):
            rospy.loginfo("{} (joint-based): {}".format(index, name))
        for (index, name) in enumerate(CARTESIAN_TRAJECTORY_CONTROLLERS):
            rospy.loginfo("{} (Cartesian): {}".format(index + len(JOINT_TRAJECTORY_CONTROLLERS), name))
        choice = -1
        while choice < 0:
            input_str = input(
                "Please choose a controller by entering its number (Enter '0' if "
                "you are unsure / don't care): "
            )
            try:
                choice = int(input_str)
                if choice < 0 or choice >= len(JOINT_TRAJECTORY_CONTROLLERS) + len(
                    CARTESIAN_TRAJECTORY_CONTROLLERS
                ):
                    rospy.loginfo(
                        "{} not inside the list of options. "
                        "Please enter a valid index from the list above.".format(choice)
                    )
                    choice = -1
            except ValueError:
                rospy.loginfo("Input is not a valid number. Please try again.")
        if choice < len(JOINT_TRAJECTORY_CONTROLLERS):
            self.joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[choice]
            return "joint_based"

        self.cartesian_trajectory_controller = CARTESIAN_TRAJECTORY_CONTROLLERS[
            choice - len(JOINT_TRAJECTORY_CONTROLLERS)
        ]
        return "cartesian"

    def switch_controller(self, target_controller):
        """Activates the desired controller and stops all others from the predefined list above"""
        other_controllers = (
            JOINT_TRAJECTORY_CONTROLLERS
            + CARTESIAN_TRAJECTORY_CONTROLLERS
            + CONFLICTING_CONTROLLERS
        )

        other_controllers.remove(target_controller)

        srv = ListControllersRequest()
        response = self.list_srv(srv)
        for controller in response.controller:
            if controller.name == target_controller and controller.state == "running":
                return

        srv = LoadControllerRequest()
        srv.name = target_controller
        self.load_srv(srv)

        srv = SwitchControllerRequest()
        srv.stop_controllers = other_controllers
        srv.start_controllers = [target_controller]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch_srv(srv)
        
    def move_once_by_joint(self, joint_position, need_time = 1):
        self.switch_controller(self.joint_trajectory_controller)
        trajectory_client = actionlib.SimpleActionClient(
            "{}/follow_joint_trajectory".format(self.joint_trajectory_controller),
            FollowJointTrajectoryAction,
        )
        timeout = rospy.Duration(5)
        if not trajectory_client.wait_for_server(timeout):
            rospy.logerr("Could not reach controller action server.")
            sys.exit(-1)
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = JOINT_NAMES

        duration_time = need_time
        point = JointTrajectoryPoint()
        point.positions = joint_position
        point.time_from_start = rospy.Duration(duration_time)
        goal.trajectory.points.append(point)
        trajectory_client.send_goal(goal)
        trajectory_client.wait_for_result()
        result = trajectory_client.get_result()
        return result

    def move_once_by_end(self, end_position, need_time = 1):
        self.switch_controller(self.cartesian_trajectory_controller)
        goal = FollowCartesianTrajectoryGoal()
        trajectory_client = actionlib.SimpleActionClient(
            "{}/follow_cartesian_trajectory".format(self.cartesian_trajectory_controller),
            FollowCartesianTrajectoryAction,
        )
        timeout = rospy.Duration(5)
        if not trajectory_client.wait_for_server(timeout):
            rospy.logerr("Could not reach controller action server.")
            sys.exit(-1)
        end_position_pose = geometry_msgs.Pose(geometry_msgs.Vector3(end_position[0], end_position[1], end_position[2]), geometry_msgs.Quaternion(end_position[3], end_position[4], end_position[5], end_position[6]))
        point = CartesianTrajectoryPoint()
        point.pose = end_position_pose
        point.time_from_start = rospy.Duration(need_time)
        goal.trajectory.points.append(point)
        trajectory_client.send_goal(goal)
        trajectory_client.wait_for_result()
        result = trajectory_client.get_result()
        return result

class ShwUr5eMoveClient:
    def __init__(self):
        rospy.init_node("move_traj_once")
        self.auto_collect_state = False
        
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
        client = TrajectoryClient(init_node = False)
        gripper = RobotiqGripper(init_node = False)
        if file is not None:
            joint_pos = np.load(file)
            if is_collect:
                self.call_auto_record_service(True)
                rospy.sleep(0.1)
            for i in range(joint_pos.shape[0]):
                client.move_once_by_joint(joint_pos[i][:6])
                rospy.sleep(0.05)
                if joint_pos[i][6]:
                    gripper.close_gripper()
                else:
                    gripper.open_gripper()
            if is_collect:
                self.call_auto_record_service(False)
        
    def end_traj_move(self, file = None, is_collect = False, add_random = False):
        client = TrajectoryClient(init_node = False)
        gripper = RobotiqGripper(init_node = False)
        if file is not None:
            end_pos = np.load(file)
            if is_collect:
                self.call_auto_record_service(True)
                rospy.sleep(0.1)
            for i in range(end_pos.shape[0]):
                client.move_once_by_end(end_pos[i][:7])
                rospy.sleep(0.05)
                if end_pos[i][7]:
                    gripper.close_gripper()
                else:
                    gripper.open_gripper()
            if is_collect:
                self.call_auto_record_service(False)

def step_npy_files_in_order(directory):
    rospy.init_node("move_traj_once")
    files = [f for f in os.listdir(directory) if f.endswith('.npy')]
    client = TrajectoryClient(init_node = False)
    client.send_init_joint_trajectory()
    gripper = RobotiqGripper(init_node = False)
    files.sort(key=lambda x: int(os.path.splitext(x)[0].split("_")[1]))
    last_gripper = 0
    now_gripper = 1
    for file in files:
        filepath = os.path.join(directory, file)
        data = np.load(filepath)
        now_gripper = data[0]
        ur_endeffector_position = data[1:8]
        ur_joint_angle = data[8:14]
        print("here!!!")
        print("ur_endeffector_position", ur_endeffector_position)
        print("ur_joint_angle", ur_joint_angle)
        # client.move_once_by_end(ur_endeffector_position)
        client.move_once_by_joint(ur_joint_angle)
        print("now_gripper", now_gripper)
        print("last_gripper", last_gripper)
        if now_gripper != last_gripper:
            if now_gripper:
                gripper.close_gripper()
            else:
                gripper.open_gripper()
            last_gripper = now_gripper
    return data
        
if __name__ == "__main__":
    # test_path = "/home/yhx/shw/src/Dataset_Collection/ABCD_all_example/traj"
    # step_npy_files_in_order("/home/yhx/shw/src/Dataset_Collection/ABCD_all_example/traj")
    run_traj = ShwUr5eMoveClient()
    control_mode = rospy.get_param('control_mode')
    trajectory_path = rospy.get_param('trajectory_path')
    is_collect = rospy.get_param('is_collect')
    if control_mode == "end":
        run_traj.end_traj_move(trajectory_path, is_collect)
    elif control_mode == "joint":
        print("sorry not test, hahaha")
        run_traj.joint_traj_move(trajectory_path, is_collect)
    else:
        print("control_mode is not correct")