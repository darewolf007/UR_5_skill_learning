sudo apt-get install ros-melodic-moveit
conda install -c conda-forge defusedxml
1. 
```
roslaunch baxter_moveit_config demo_baxter.launch load_robot_description:=true right_electric_gripper:=true left_electric_gripper:=true
```
2. 
```
rosrun baxter_tools enable_robot.py -e
rosrun baxter_interface joint_trajectory_action_server.py
```

# MoveIt! Robots

This repository contains configuration packages for different robots using MoveIt!

[![Build Status](https://travis-ci.org/ros-planning/moveit_robots.svg?branch=master)](https://travis-ci.org/ros-planning/moveit_robots)

Guidelines
---------

Add your robot's MoveIt configuration package to the root of this repository.

Name the package YOURROBOT_moveit_config... for example 'pr2_moveit_config'

Add a README.md file inside your robot's config package that explains where and how to get your robot's URDF file - i.e. where to download the corresponding YOURROBOT_description package that contains your robot's URDF file.
