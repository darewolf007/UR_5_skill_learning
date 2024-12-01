# UR5e

## Introduction

All packages(no eyehand package, some bug happened when compiled easy_handeye by catkin_make_isolated).

**if you want to use joint control, please check JOINT_NAMES!!!**

For example, when using a command(rostopic echo /joint_states), the output joint sequence order of the result is inconsistent with the official code.
(should be: elbow_joint, shoulder_lift_joint, shoulder_pan_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint)

**if you want to control robotiq gripper though UR control cabinet**

Replace 
```
from utils.robotiq_gripper import RobotiqGripper 
```
to 
```
from utils.robotiq_gripper_remote import RobotiqGripper
```
## launch file(all new terminal need to source devel_isolated/setup.bash)

### UR5e control(launch: camera, robotiq, UR, moveit)
#### control robotiq gripper though computer

```
roslaunch Move_UR init_ur_robot.launch local_gripper_communicate:=True
```

#### control robotiq gripper though ur control cabinet

```
roslaunch Move_UR init_ur_robot.launch local_gripper_communicate:=False
```

### Move robot follow a trajectory and collect data(data_collection env: py27)

**you need to set trajectory_path first, on launch file args or terminal**

eg: roslaunch Move_UR move_follow_trajectory.launch is_collect:=True control_mode:=end trajectory_path:="your trajectory path"

```
roslaunch Move_UR move_follow_trajectory.launch
roslaunch Move_UR data_collection.launch
```

### Only move robot follow a trajectory

### Only move robot follow a trajectory under end control

```
roslaunch Move_UR move_follow_trajectory.launch is_collect:=False local_gripper_communicate:=False
```

### Only move robot follow a trajectory under joint control

```
roslaunch Move_UR move_follow_trajectory.launch is_collect:=False control_mode:=joint
```

### Only data collection without move robot(data_collection env: py27)

```
roslaunch Move_UR data_collection.launch
rosservice call /collect_bool_service True
```

### Robotiq control on terminal(sudo chmod 777 /dev/ttyUSB0)

```
roslaunch Move_UR control_robotiq.launch
```

### Camera

```
roslaunch azure_kinect_ros_driver driver.launch
```

### algorithm(visual-->action) network communication

```
roslaunch Move_UR move_follow_action.launch
python src/Move_UR/scripts/publish_action_client_network.py
```

### algorithm(visual-->action) local communication

```
python src/Move_UR/scripts/publish_action_client_local.py
```

### Handeye calibration(need to cd shw_eyehand)

```
source devel/setup.bash
roslaunch easy_handeye ur5_kinect_calibration.launch
```

## tool box

### collect step rgbd image and robot state info(env: py27 & other args: --base_data_path your_path)

```
source devel_isolated/setup.bash
python src/Move_UR/scripts/useful_tool/collect_image_state.py
```

### collect robot keypoint or entire trajectory(env: regnet & other args: --base_data_path your_path)

#### collect robot entire trajectory(/tf hz is 560~570)

```
python src/Move_UR/scripts/useful_tool/collect_robot_trajectory.py --mode record --record_step 1000
```

#### collect robot keypoint trajectory

```
python src/Move_UR/scripts/useful_tool/collect_robot_trajectory.py --mode select
```

### Get keypoints from keyboard(env: regnet)

```
python src/Move_UR/scripts/useful_tool/collect_keypose.py
```

## TODO

* [ ]  refactor python file except move_ur_follow_trajectory
* [ ]  add internet communication on  remote workstationown
* [ ]  test internet communication on  robot
* [ ]  test robotiq remote control and change the code
