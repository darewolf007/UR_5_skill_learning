# UR5e
## Introduction
    All projects(no eyehand).
## launch file
### UR5e control
roslaunch Move_UR init_ur_robot.launch
### Move robot follow a trajectory
roslaunch Move_UR move_follow_traj.launch
### Data collection
roslaunch Move_UR data_collection.launch (py27 env)
### Robotiq control on terminal(sudo chmod 777 /dev/ttyUSB0)
roslaunch Move_UR control_robotiq.launch
### Get keypoints from keyboard
python src/Move_UR/scriptscollect_keypose.py (regnet env)

