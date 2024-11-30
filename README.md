# UR5e
## Introduction
    All projects(no eyehand).
## launch file
### UR5e control(launch: camera, robotiq, UR, moveit)
roslaunch Move_UR init_ur_robot.launch
### Move robot follow a trajectory
roslaunch Move_UR move_follow_traj.launch 
### Data collection
roslaunch Move_UR data_collection.launch (py27 env)
### Robotiq control on terminal(sudo chmod 777 /dev/ttyUSB0)
roslaunch Move_UR control_robotiq.launch
### Get keypoints from keyboard
python src/Move_UR/scripts/collect_keypose.py (regnet env)
### Handeye calibration(shw_eyehand)
roslaunch easy_handeye ur5_kinect_calibration.launch
### Camera
roslaunch azure_kinect_ros_driver driver.launch
### algorithm(visual-->action) communication
roslaunch Move_UR move_follow_action.launch
python src/Move_UR/scripts/publish_action_client_network.py 

