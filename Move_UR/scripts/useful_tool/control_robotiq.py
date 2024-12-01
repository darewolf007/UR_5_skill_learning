#!/usr/bin/env python2
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
import rospy
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_input

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