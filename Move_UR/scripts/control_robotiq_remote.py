#!/usr/bin/env python2
from utils.robotiq_gripper_remote import RobotiqGripper

if __name__ == '__main__':
    import rospy
    while not rospy.is_shutdown():
        rospy.init_node("remote gripper control")
        robot_ip = rospy.get_param('~robot_ip')
        gripper = RobotiqGripper(robot_ip)
        gripper.gen_gripper_command(gripper.askForCommand())
        rospy.sleep(0.1)