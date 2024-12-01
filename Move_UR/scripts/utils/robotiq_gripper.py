#!/usr/bin/env python2
 
from __future__ import print_function
import sys
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
import rospy
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from time import sleep
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolRequest
# Compatibility for python2 and python3
if sys.version_info[0] < 3:
    input = raw_input

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
    
    def _collect_gripper_state(self, command):
        if command.gPR == 0:
            self.gripper_state = False
        else:
            self.gripper_state = True
    
    def get_gripper_state(self):
        return self.gripper_state
    
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