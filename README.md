# UR5e
## Introduction
    All packages(no eyehand package, some bug happened when easy_handeye  compiled by catkin_make_isolated).
    if you want to use joint control, please check JOINT_NAMES!!!
    For example, when using a command(rostopic echo /joint_states), the output joint sequence order of the result is inconsistent with the official code.
    (should be: elbow_joint, shoulder_lift_joint, shoulder_pan_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint)
## launch file(all commends need to source devel_isolated/setup.bash)
### UR5e control(launch: camera, robotiq, UR, moveit)
roslaunch Move_UR init_ur_robot.launch
### Move robot follow a trajectory and collect data
roslaunch Move_UR move_follow_traj.launch 
roslaunch Move_UR data_collection.launch (py27 env)
### Data collection without move robot
roslaunch Move_UR data_collection.launch (py27 env)
rosservice call /collect_bool_service True
### Robotiq control on terminal(sudo chmod 777 /dev/ttyUSB0)
roslaunch Move_UR control_robotiq.launch
### Get keypoints from keyboard
python src/Move_UR/scripts/collect_keypose.py (regnet env)
### Camera
roslaunch azure_kinect_ros_driver driver.launch
### algorithm(visual-->action) network communication
roslaunch Move_UR move_follow_action.launch
python src/Move_UR/scripts/publish_action_client_network.py 
### algorithm(visual-->action) local communication
python src/Move_UR/scripts/publish_action_client_local.py 
### Handeye calibration(need to cd shw_eyehand)
source devel/setup.bash
roslaunch easy_handeye ur5_kinect_calibration.launch
