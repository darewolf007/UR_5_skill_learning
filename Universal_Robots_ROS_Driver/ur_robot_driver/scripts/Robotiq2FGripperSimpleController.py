#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Robotiq, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotiq, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2012, Robotiq, Inc.
# Revision $Id$

"""@package docstring
Command-line interface for sending simple commands to a ROS node controlling a 2F gripper.

This serves as an example for publishing messages on the 'Robotiq2FGripperRobotOutput' topic using the 'Robotiq2FGripper_robot_output' msg type for sending commands to a 2F gripper.
"""

import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
import rospy
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from time import sleep
grip_pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)

def genCommand(char, command):
    """Update the command according to the character entered by the user."""    
        
    if char == 'a':
        command = outputMsg.Robotiq2FGripper_robot_output();
        command.rACT = 1
        command.rGTO = 1
        command.rSP  = 225
        command.rFR  = 175

    if char == 'r':
        command = outputMsg.Robotiq2FGripper_robot_output();
        command.rACT = 0

    if char == 'c':
        command.rPR = 255

    if char == 'o':
        command.rPR = 0   

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
        

# def askForCommand(command):
#     """Ask the user for a command to send to the gripper."""    

#     currentCommand  = 'Simple 2F Gripper Controller\n-----\nCurrent command:'
#     currentCommand += '  rACT = '  + str(command.rACT)
#     currentCommand += ', rGTO = '  + str(command.rGTO)
#     currentCommand += ', rATR = '  + str(command.rATR)
#     currentCommand += ', rPR = '   + str(command.rPR )
#     currentCommand += ', rSP = '   + str(command.rSP )
#     currentCommand += ', rFR = '   + str(command.rFR )


#     print currentCommand

#     strAskForCommand  = '-----\nAvailable commands\n\n'
#     strAskForCommand += 'r: Reset\n'
#     strAskForCommand += 'a: Activate\n'
#     strAskForCommand += 'c: Close\n'
#     strAskForCommand += 'o: Open\n'
#     strAskForCommand += '(0-255): Go to that position\n'
#     strAskForCommand += 'f: Faster\n'
#     strAskForCommand += 'l: Slower\n'
#     strAskForCommand += 'i: Increase force\n'
#     strAskForCommand += 'd: Decrease force\n'
    
#     strAskForCommand += '-->'

#     return raw_input(strAskForCommand)
command_list = ['r','a','c','o']

def publisher():
    """Main loop which requests new commands and publish them on the Robotiq2FGripperRobotOutput topic."""
    rospy.init_node('Robotiq2FGripperRobotOutput')
    
    pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)

    command = outputMsg.Robotiq2FGripper_robot_output();

    while not rospy.is_shutdown():
        # command = genCommand(askForCommand(command), command) 
        command_list = ["r","a","c","o"]            
        command = genCommand(command_list[0], command)
        pub.publish(command)
        command = genCommand(command_list[1], command)
        pub.publish(command)
        # command = genCommand(command_list[2], command)
        # pub.publish(command)
        # command = genCommand(command_list[3], command)
        # pub.publish(command)
        # rospy.sleep(0.1)


def robotiq_reset():
    """Main loop which requests new commands and publish them on the Robotiq2FGripperRobotOutput topic."""
    rospy.init_node('Robotiq2FGripperRobotOutput')
    
    # pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)

    command = outputMsg.Robotiq2FGripper_robot_output();
    i = 0
    print (rospy.is_shutdown())

    while not rospy.is_shutdown():
        i= i+1
        print(i)
    # command = genCommand(askForCommand(command), command) 
           
        command = genCommand(command_list[0], command)
        grip_pub.publish(command)
        rospy.sleep(0.5)
        if i > 1:
            break
    


        # command = genCommand(askForCommand(command), command)           
        # command = genCommand(command_list[0], command)
        # pub.publish(command)
    command = genCommand(command_list[1], command)
    grip_pub.publish(command)
    rospy.sleep(1.5)
    # command = genCommand(command_list[2], command)
    # pub.publish(command)
    # rospy.sleep(3)
    # command = genCommand(command_list[3], command)
    # pub.publish(command)
    return command

def robotiq_close(command, pos):
    # rospy.init_node('Robotiq2FGripperRobotOutput')
    command_list = ['r','a','c','o']  
    # pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)
    if pos == 255:
 
        command = genCommand(command_list[2], command)
        grip_pub.publish(command)
        rospy.sleep(1.5)
    else:
        command = genCommand(pos, command)
        grip_pub.publish(command)
        rospy.sleep(1.5) 


def robotiq_open(command):
    # rospy.init_node('Robotiq2FGripperRobotOutput')
  
    # pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)

    command_list = ['r','a','c','o']
    command = genCommand(command_list[3], command)
    grip_pub.publish(command)
    rospy.sleep(1.5)


                            

if __name__ == '__main__':
    reset1()
    # reset2()
    # publisher()

