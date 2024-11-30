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

    def send_init_joint_trajectory(self, joint_pos = [1.542957607899801, -1.3407166761210938, 1.2018647193908691, -1.75225891689443, -1.6117513815509241, -0.3191445509540003]):
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
        duration_list = [1.0]
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

    def send_init_cartesian_trajectory(self, end_pos = [0.249179509371,-0.392681547898, 0.38588219159, 0.927532761465,0.368790596434,0.0409693418325,0.044698830199]):
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

        # duration_list = [3.0, 4.0, 5.0, 6.0, 7.0]
        duration_list = [2.0]
        for i, pose in enumerate(pose_list):
            point = CartesianTrajectoryPoint()
            point.pose = pose
            point.time_from_start = rospy.Duration(duration_list[i])
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
        # The following list are arbitrary positions
        # Change to your own needs if desired
        # position_list = [[1.542957607899801, -1.3407166761210938, 1.2018647193908691, -1.75225891689443, -1.6117513815509241, -0.3191445509540003]]
        # duration_list = [3.0]
        # for i, position in enumerate(position_list):
        #     point = JointTrajectoryPoint()
        #     point.positions = position
        #     point.time_from_start = rospy.Duration(duration_list[i])
        #     goal.trajectory.points.append(point)
        trajectory_client.send_goal(goal)
        trajectory_client.wait_for_result()
        result = trajectory_client.get_result()
        return result

    def move_once_by_end(self, end_position):
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

        # pose_list = [
        #     geometry_msgs.Pose(
        #         geometry_msgs.Vector3(0.249179509371,-0.392681547898, 0.38588219159), geometry_msgs.Quaternion(0.927532761465,0.368790596434,0.0409693418325,0.044698830199) 
        #     ),
        # ]

        # duration_list = [3.0, 4.0, 5.0, 6.0, 7.0]
        duration_time = 1.7
        end_position_pose = geometry_msgs.Pose(geometry_msgs.Vector3(end_position[0], end_position[1], end_position[2]), geometry_msgs.Quaternion(end_position[3], end_position[4], end_position[5], end_position[6]))
        point = CartesianTrajectoryPoint()
        point.pose = end_position_pose
        point.time_from_start = rospy.Duration(duration_time)
        goal.trajectory.points.append(point)
        # for i, pose in enumerate(pose_list):
        #     point = CartesianTrajectoryPoint()
        #     point.pose = pose
        #     point.time_from_start = rospy.Duration(duration_list[i])
        #     goal.trajectory.points.append(point)
        trajectory_client.send_goal(goal)
        trajectory_client.wait_for_result()
        result = trajectory_client.get_result()
        return result

class shw_ur_auto_collect:
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
    
    def shw_joint_traj_move(self):
        position1 = [1.466440502797262, -1.2794888776591797, 1.0327072143554688, -1.7451826534666957, -1.6147378126727503, -0.4882739225970667]
        position2 = [1.6689704100238245, -1.182669000034668, 1.032731533050537, -2.044462820092672, -1.6152775923358362, -0.48686296144594365]
        position3 = [1.4728835264789026, -1.0546439451030274, 1.0802836418151855, -1.9741212330260218, -1.6147378126727503, -0.439035717641012]
        position4 = [1.5496943632708948, -1.1050432485393067, 1.2299962043762207, -1.993239542047018, -1.611727539693014, -0.2893560568438929]
        position5 = [1.9218161741839808, -1.3694866460612793, 1.1515707969665527, -2.1046015224852503, -1.613154713307516, -0.36831933656801397]
        position5_1 = [1.884165112172262, -1.3399351400187989, 1.1619720458984375, -2.095889230767721, -1.6129987875567835, -0.3578456083880823]
        position_B_No_A_1 = [0.8840902487384241, -0.8947866720012208, 1.3807368278503418, -1.5311687749675293, -1.6080811659442347, -0.13969737688173467]
        position_B_No_A_2 = [1.036532227193014, -0.8801983159831543, 1.3847107887268066, -1.6980506382384242, -1.6079614798175257, -0.134958569203512]
        position6 = [1.7884085814105433, -1.4583032888225098, 1.1515822410583496, -1.8824826679625453, -1.6126988569842737, -0.3694108168231409]
        position7 = [1.4580672422992151, -1.2002991002849122, 1.3687920570373535, -1.8002673588194789, -1.607781712208883, -0.1514514128314417]
        position8 = [1.4957850615130823, -1.1547544759562989, 1.3712100982666016, -1.8833338222899378, -1.6078417936908167, -0.14855462709535772]
        position9 = [1.3603995482074183, -1.1997497838786622, 1.3712220191955566, -1.702975412408346, -1.6076377073871058, -0.14942866960634404]
        position10 = [1.334411923085348, -1.1784404081157227, 1.6072869300842285, -1.7156340084471644, -1.6007416884051722, 0.08688497543334961]
        position11 = [1.4255526701556605, -1.153548077946045, 1.6072869300842285, -1.8316661320128382, -1.6008132139789026, 0.08745908737182617]
        position12 = [1.520700756703512, -1.1125562948039551, 1.5936856269836426, -1.9682699642577113, -1.6012089888202112, 0.07445669174194336]
        position13 = [1.3677924315081995, -1.1778314870646973, 1.5936493873596191, -1.7500716648497523, -1.6008732954608362, 0.07341480255126953]
        position14 = [1.0905121008502405, -0.9957450193217774, 1.885277271270752, -1.6450392208495082, -1.5965197722064417, 0.36530590057373047]
        position15 = [0.9415224234210413, -0.9991100591472168, 1.8852410316467285, -1.4927070897868653, -1.5965436140643519, 0.3646969795227051]
        position16 = [1.2276843229876917, -1.188760594730713, 1.6101393699645996, -1.5985661945738734, -1.6006577650653284, 0.08925580978393555]
        position17 = [1.4684203306781214, -1.1329916280559083, 1.6102352142333984, -1.8948952160277308, -1.6009214560138147, 0.0906682014465332]
        position18 =  [1.2855818907367151, -1.0156710904887696, 1.828209400177002, -1.8219796619811, -1.597670857106344, 0.3088383674621582]
        client = TrajectoryClient(init_node = False)
        gripper = RobotiqGripper(init_node = False)
        client.send_init_joint_trajectory()
        self.call_auto_record_service(True)
        rospy.sleep(0.1)
        client.move_once_by_joint(position1)
        client.move_once_by_joint(position2)
        gripper.close_gripper()
        client.move_once_by_joint(position3)
        client.move_once_by_joint(position4)
        client.move_once_by_joint(position5)
        client.move_once_by_joint(position5_1)
        gripper.open_gripper()
        client.move_once_by_joint(position6)
        client.move_once_by_joint(position7)
        client.move_once_by_joint(position8)
        # task B with out A
        # client.move_once_by_joint(position_B_No_A_1, need_time= 3)
        # client.move_once_by_joint(position_B_No_A_2, need_time= 1)
        gripper.close_gripper()
        client.move_once_by_joint(position9)
        client.move_once_by_joint(position10)
        client.move_once_by_joint(position11)
        gripper.open_gripper()
        # task C with out B
        client.move_once_by_joint(position6) 
        client.move_once_by_joint(position11, need_time= 5)
        #####
        client.move_once_by_joint(position12)
        gripper.close_gripper()
        client.move_once_by_joint(position13)
        client.move_once_by_joint(position14)
        gripper.open_gripper()
        client.move_once_by_joint(position15)
        client.move_once_by_joint(position16)
        client.move_once_by_joint(position17)
        gripper.close_gripper()
        client.move_once_by_joint(position18)
        client.send_init_joint_trajectory()
        gripper.open_gripper()
        rospy.sleep(0.5)
        self.call_auto_record_service(False)
        
    def shw_end_traj_move(self):
        #TODO   
        end_position1 = np.array([ -0.206992924455, -0.578936818643, 0.195838372629, -0.0233070786438, -0.99939292491, -0.0180450417058, 0.0185725118253])
        end_position2 = np.array([-0.206991188991, -0.578940572239, 0.0716936458811, -0.0233230925389, -0.999392838978, -0.0180389129846, 0.0185629839498])
        end_position3 = np.array([-0.20700119726, -0.641233103889, 0.0717359339521, -0.0233226724054, -0.999393053353, -0.0179992879558, 0.0185904140367])
        end_position4 = np.array([-0.0998959010988, -0.641210695104, 0.0717140811463, -0.0233168900578, -0.999392567244, -0.0180238577497, 0.0185999926532])
        end_position5 = np.array([-0.09991263381, -0.516755156918, 0.0717003860638, -0.0233413707661, -0.999391792864, -0.0180392044889, 0.0185960175454])
        end_position5_1 = np.array([-0.0998775515271, -0.530433449629, 0.0716401283034, -0.0232925413361, -0.999392380754, -0.0181039222172, 0.0185627263313])
        end_position6 = np.array([-0.0998838243869, -0.516728602821, 0.156206355017,  -0.0233066130699, -0.999392522828,  -0.0180798482429, 0.0185608779277])
        end_position7 = np.array([-0.00648704385, -0.635758311955, 0.156234591449,  -0.0233307172244, -0.999392618697, -0.0180355201154, 0.0185685578787])
        end_position8 = np.array([-0.00654007907002, -0.643780357407, 0.117624929982, -0.0233319170712, -0.99939205311, -0.0180465275747, 0.0185867874801])
        end_position9 = np.array([-0.00652495655719, -0.643757624262, 0.193404205376, -0.023330213161, -0.999391427116, -0.0180661378935, 0.0186035271757])
        end_position10 = np.array([0.14687754329, -0.641263574685, 0.193443435189, -0.0235856263186, -0.9995432427, -0.00474075782972, 0.0182906921608])
        end_position11 = np.array([0.146877710209, -0.641253247183, 0.144769143385, -0.0235860662888, -0.999543434216, -0.00476913763771, 0.0182722718409])
        end_position12 = np.array([0.138062452332,  -0.641246397819, 0.0871108380974, -0.0235636016087, -0.999543786525, -0.00477654649921, 0.0182800465413])
        end_position13 = np.array([0.138046127537, -0.641270231043, 0.180231794064, -0.0235812591532, -0.999543413749, -0.00475381715455, 0.0182835847198])
        end_position14 = np.array([0.340197190592, -0.641253071988, 0.180205213314, -0.0235820282926, -0.999542986058, -0.00479392217152, 0.018295498722])
        end_position15 = np.array([0.340220117798, -0.641236669639, 0.240721921033, -0.0235595648436, -0.999543849053, -0.00477777639142, 0.0182815091761])
        end_position16 = np.array([0.148797377347, -0.641256149283, 0.240741409963, -0.0235455029645, -0.999544339982, -0.00476253226278, 0.0182767608072])
        end_position17 = np.array([0.148784712853, -0.641270187732, 0.117427201769, -0.023576366527, -0.999543950287, -0.00474363197101, 0.0182631963737])
        end_position18 = np.array([0.297732688926, -0.641258303198, 0.117430366838, -0.0235777550891, -0.999543680571, -0.00475943513663, 0.018272051672])
        client = TrajectoryClient(init_node = False)
        gripper = RobotiqGripper(init_node = False)
        client.send_init_joint_trajectory()
        self.call_auto_record_service(True)
        rospy.sleep(0.1)
        end_position1[:3] += np.random.uniform(low= 0, high=0.05, size = (3,))
        client.move_once_by_end(end_position1)
        end_position2[:3] += np.random.uniform(low= -0.01, high=0.01, size = (3,))
        # end_position2[:2] += np.random.uniform(low= -0.02, high=0.02, size = (2,))
        client.move_once_by_end(end_position2)
        gripper.close_gripper()
        end_position3[:3] += np.random.uniform(low= -0.01, high=0.01, size = (3,))
        client.move_once_by_end(end_position3)
        end_position4[:2] -= np.random.uniform(low= 0, high=0.02, size = (2,))
        end_position4[0] += np.random.uniform(low= 0, high=0.03, size = (1,))
        client.move_once_by_end(end_position4)
        important_noise = np.random.uniform(low= 0, high=0.02, size = (2,))
        end_position5[:2] -= important_noise
        client.move_once_by_end(end_position5)
        end_position5_1[:2] -= np.random.uniform(low= 0, high=0.05, size = (2,))
        end_position5_1[0] -= np.random.uniform(low= 0.02, high=0.08, size = (1,))
        client.move_once_by_end(end_position5_1)
        gripper.open_gripper()
        
        end_position6[2] += np.random.uniform(low=-0.02, high=0.02, size = (1,))
        client.move_once_by_end(end_position6)
        end_position7[:2] -= np.random.uniform(low= 0, high=0.025, size = (2,))
        client.move_once_by_end(end_position7)
        end_position8[:3] -= np.random.uniform(low= 0, high=0.01, size = (3,))
        end_position8[2] += np.random.uniform(low= -0.02, high=0, size = (1,))
        client.move_once_by_end(end_position8)
        gripper.close_gripper()
        
        
        important_random_2 = np.random.uniform(low= -0.005, high=0.005, size = (3,))
        end_position9[:3] += important_random_2
        client.move_once_by_end(end_position9)
        end_position10[:3] -= important_random_2
        client.move_once_by_end(end_position10)
        end_position11[2] += np.random.uniform(low= -0.02, high=0, size = (1,))
        client.move_once_by_end(end_position11)
        gripper.open_gripper()
        
        end_position12[2] += np.random.uniform(low= -0.02, high=0.01, size = (1,))
        client.move_once_by_end(end_position12)
        gripper.close_gripper()
        end_position13[:3] += np.random.uniform(low= 0, high=0.03, size = (3,))
        end_position13[1] -= np.random.uniform(low= 0, high=0.03, size = (1,))
        client.move_once_by_end(end_position13)
        end_position14[0] += np.random.uniform(low= 0.0, high=0.01, size = (1,))
        end_position14[1] -= np.random.uniform(low= 0.01, high=0.02, size = (1,))
        end_position14[:2] += np.random.uniform(low= 0.01, high=0.02, size = (2,))
        end_position14[2] +=np.random.uniform(low= -0.01, high=0, size = (1,))
        client.move_once_by_end(end_position14)
        gripper.open_gripper()
        
        end_position15[1] += np.random.uniform(low= 0, high=0.02, size = (1,))
        end_position15[1:3] += np.random.uniform(low= 0, high=0.06, size = (2,))
        client.move_once_by_end(end_position15)
        end_position16[:3] -= np.random.uniform(low= 0, high=0.05, size = (3,))
        client.move_once_by_end(end_position16)
        end_position17[:3] -= np.random.uniform(low= 0, high=0.05, size = (3,))
        client.move_once_by_end(end_position17)
        gripper.close_gripper()
        end_position18[:3] += np.random.uniform(low= -0.03, high=0.03, size = (3,))
        end_position18[2] += np.random.uniform(low= 0.00, high=0.01, size = (1,))
        end_position18[0] += np.random.uniform(low= 0.02, high=0.03, size = (1,))
        client.move_once_by_end(end_position18)
        client.send_init_joint_trajectory()
        gripper.open_gripper()
        rospy.sleep(0.5)
        self.call_auto_record_service(False)

def test_npy_files_in_order(directory):
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
    # test_npy_files_in_order("/home/yhx/shw/src/Dataset_Collection/ABCD_all_example/traj")
    run_traj = shw_ur_auto_collect()
    run_traj.shw_joint_traj_move()
    # run_traj.shw_end_traj_move()
    # gripper = RobotiqGripper(init_node = True)