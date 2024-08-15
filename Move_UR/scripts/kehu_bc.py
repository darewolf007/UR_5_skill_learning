#!/usr/bin/env python2
 
from __future__ import print_function

import sys
import rospy
# from beginner_tutorials.srv import *
 
from hh import get_new_pos
import numpy as np
import os
#
# Code adapted from https://github.com/ros-planning/moveit_tutorials/blob/kinetic-devel/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py
# to UR5 robot
#
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

# sys.path.append('/home/yhx/Documents/catkin_ws/src/Universal_Robots_ROS_Driver/ur_robot_driver/scripts')
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

# Compatibility for python2 and python3
if sys.version_info[0] < 3:
    input = raw_input

# If your robot description is created with a tf_prefix, those would have to be adapted
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
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


class TrajectoryClient:
    """Small trajectory client to test a joint trajectory"""

    def __init__(self):

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

    def send_joint_trajectory(self):
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
        position_list = [[0, -1.57, -1.57, 0, 0, 0]]
        position_list.append([0.2, -1.57, -1.57, 0, 0, 0])
        position_list.append([-0.5, -1.57, -1.2, 0, 0, 0])
        duration_list = [3.0, 7.0, 10.0]
        for i, position in enumerate(position_list):
            point = JointTrajectoryPoint()
            point.positions = position
            point.time_from_start = rospy.Duration(duration_list[i])
            goal.trajectory.points.append(point)

        self.ask_confirmation(position_list)
        rospy.loginfo("Executing trajectory using the {}".format(self.joint_trajectory_controller))

        trajectory_client.send_goal(goal)
        trajectory_client.wait_for_result()

        result = trajectory_client.get_result()
        rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))

    def send_cartesian_trajectory(self, next_pos):
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
                # geometry_msgs.Vector3(0.7+0.05*(numpy.random.random()-0.5), -0.133+0.05*(numpy.random.random()-0.5), 0.258+0.05*(numpy.random.random()-0.5)), geometry_msgs.Quaternion(0.707, -0.707,0,0) #(0100)
                geometry_msgs.Vector3(next_pos[0], next_pos[1], next_pos[2]), geometry_msgs.Quaternion(next_pos[3], next_pos[4],next_pos[5],next_pos[6]) #(0100)
            
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

    ###############################################################################################
    #                                                                                             #
    # Methods defined below are for the sake of safety / flexibility of this demo script only.    #
    # If you just want to copy the relevant parts to make your own motion script you don't have   #
    # to use / copy all the functions below.                                                       #
    #                                                                                             #
    ###############################################################################################

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
        choice = 7
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

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True



class MoveGroupTutorial(object):
    """MoveGroupTutorial"""
    def __init__(self):
        super(MoveGroupTutorial, self).__init__()

        # First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node('move_group_tutorial_ur5e', anonymous=True)

        # Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
        # the robot:
        robot = moveit_commander.RobotCommander()

        # Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
        # to the world surrounding the robot:
        scene = moveit_commander.PlanningSceneInterface()

        # Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        # to one group of joints.  In this case the group is the joints in the UR5
        # arm so we set ``group_name = manipulator``. If you are using a different robot,
        # you should change this value to the name of your robot arm planning group. 
        group_name = "manipulator" # See .srdf file to get available group names
        group = moveit_commander.MoveGroupCommander(group_name)

        group.set_planner_id("TRRTkConfigDefault")

        # current_joint_values = group.get_current_joint_values()
        # joint_constraints = Constraints()
        
        # for i, joint_name in enumerate(group.get_active_joints()):
        #     # Example joint constraint for each joint (replace with your joint limits)
        #         joint_constraint = JointConstraint(
        #             joint_name=joint_name,
        #             position=current_joint_values[i],
        #             tolerance_above=5,  # Replace with your desired tolerance above
        #             tolerance_below=5,  # Replace with your desired tolerance below
        #             weight=1.0  # Weight of the constraint
        #         )

        #         # Add the joint constraint to the Constraints object
        #         joint_constraints.joint_constraints.append(joint_constraint)

        # # Set the joint constraints for the motion planning
        # group.set_path_constraints(joint_constraints)
        
        # We create a `DisplayTrajectory`_ publisher which is used later to publish
        # trajectories for RViz to visualize:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

        # We can get the name of the reference frame for this robot:
        planning_frame = group.get_planning_frame()
        # print "============ Reference frame: %s" % planning_frame

        # We can also print the name of the end-effector link for this group:
        eef_link = group.get_end_effector_link()
        # print "============ End effector: %s" % eef_link

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        # print "============ Robot Groups:", robot.get_group_names()

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        # print "============ Printing robot state"
        # print robot.get_current_state()
        # print ""

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        # ipdb.set_trace()
        self.group_names = group_names
        # self.go_to_pose_goal()

    def go_to_up_state(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group

        joint_goal = group.get_current_joint_values()
        print(type(joint_goal), joint_goal)

        # joint goal is a list of 7 elements : (x,y,z,qx,qy,qz,qw) can be composed of pose_msg
        joint_goal[0] = 0
        joint_goal[1] = -pi * 0.5
        joint_goal[2] = 0
        joint_goal[3] = -pi * 0.5
        joint_goal[4] = 0
        joint_goal[5] = 0    

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        group.stop()

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self, next_pos):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group

        current_pose = group.get_current_pose().pose
        print("Current pose: ", current_pose)
        # ipdb.set_trace()
        # We can plan a motion for this group to a desired pose for the end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = next_pos[3]
        pose_goal.orientation.y = next_pos[4]
        pose_goal.orientation.z = next_pos[5]
        pose_goal.orientation.w = next_pos[6]

        pose_goal.position.x = next_pos[0]
        pose_goal.position.y = next_pos[1]
        pose_goal.position.z = next_pos[2]
        group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        plan = group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        group.clear_pose_targets()

        current_pose = group.get_current_pose().pose
        print("New current pose: ", current_pose)

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)


    def plan_cartesian_path(self, scale=1):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group

        current_pose = group.get_current_pose().pose
        print("Current pose: ", current_pose)

        # Cartesian Paths
        # You can plan a Cartesian path directly by specifying a list of waypoints
        # for the end-effector to go through:
        waypoints = []
        0.6815, 0.0061, 0.0412, geometry_msgs.Quaternion(0.707, -0.707,0,0)

        wpose = group.get_current_pose().pose
        wpose.position.x = 0.2  
        wpose.position.y = 0.01
        wpose.position.z = 0.2
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y += 0.1
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= 0.2
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
        (plan, fraction) = group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        # Displaying a Trajectory
        # You can ask RViz to visualize a plan (aka trajectory) for you. But the
        # group.plan() method does this automatically so this is not that useful
        # here (it just displays the same trajectory again):
        #
        # A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        # We populate the trajectory_start with our current robot state to copy over
        # any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory);

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group

        # Executing a Plan
        # Use execute if you would like the robot to follow
        # the plan that has already been computed:
        group.execute(plan, wait=True)

        # **Note:** The robot's current joint state must be within some tolerance of the
        # first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail

    # def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    #     # Copy class variables to local variables to make the web tutorials more clear.
    #     # In practice, you should use the class variables directly unless you have a good
    #     # reason not to.
    #     box_name = self.box_name
    #     scene = self.scene

    #     ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    #     ##
    #     ## Ensuring Collision Updates Are Receieved
    #     ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    #     ## If the Python node dies before publishing a collision object update message, the message
    #     ## could get lost and the box will not appear. To ensure that the updates are
    #     ## made, we wait until we see the changes reflected in the
    #     ## ``get_known_object_names()`` and ``get_known_object_names()`` lists.
    #     ## For the purpose of this tutorial, we call this function after adding,
    #     ## removing, attaching or detaching an object in the planning scene. We then wait
    #     ## until the updates have been made or ``timeout`` seconds have passed
    #     start = rospy.get_time()
    #     seconds = rospy.get_time()
    #     while (seconds - start < timeout) and not rospy.is_shutdown():
    #         # Test if the box is in attached objects
    #         attached_objects = scene.get_attached_objects([box_name])
    #         is_attached = len(attached_objects.keys()) > 0

    #     # Test if the box is in the scene.
    #     # Note that attaching the box will remove it from known_objects
    #     is_known = box_name in scene.get_known_object_names()

    #     # Test if we are in the expected state
    #     if (box_is_attached == is_attached) and (box_is_known == is_known):
    #         return True

    #     # Sleep so that we give other threads time on the processor
    #     rospy.sleep(0.1)
    #     seconds = rospy.get_time()

    #     # If we exited the while loop without returning then we timed out
    #     return False
    #     ## END_SUB_TUTORIAL

    # def add_box(self, timeout=4):
    #     # Copy class variables to local variables to make the web tutorials more clear.
    #     # In practice, you should use the class variables directly unless you have a good
    #     # reason not to.
    #     box_name = self.box_name
    #     scene = self.scene

    #     ## BEGIN_SUB_TUTORIAL add_box
    #     ##
    #     ## Adding Objects to the Planning Scene
    #     ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    #     ## First, we will create a box in the planning scene at the location of the left finger:
    #     box_pose = geometry_msgs.msg.PoseStamped()
    #     box_pose.header.frame_id = "panda_leftfinger"
    #     box_pose.pose.orientation.w = 1.0
    #     box_name = "box"
    #     scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

    #     ## END_SUB_TUTORIAL
    #     # Copy local variables back to class variables. In practice, you should use the class
    #     # variables directly unless you have a good reason not to.
    #     self.box_name=box_name
    #     return self.wait_for_state_update(box_is_known=True, timeout=timeout)


    def attach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        ## BEGIN_SUB_TUTORIAL attach_object
        ##
        ## Attaching Objects to the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
        ## robot be able to touch them without the planning scene reporting the contact as a
        ## collision. By adding link names to the ``touch_links`` array, we are telling the
        ## planning scene to ignore collisions between those links and the box. For the Panda
        ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
        ## you should change this value to the name of your end effector group name.
        grasping_group = 'hand'
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link

        ## BEGIN_SUB_TUTORIAL detach_object
        ##
        ## Detaching Objects from the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def remove_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL remove_object
        ##
        ## Removing Objects from the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can remove the box from the world.
        scene.remove_world_object(box_name)

        ## **Note:** The object must be detached before we can remove it from the world
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)



class RecordROS():
    def __init__(self):
        self.cvbridge = CvBridge()
        self.count = 0
        self.is_save = False
        self.images = None
        self.ps_msg = None
        self.obs_arr = list()
        self.state_arr = list()
        rospy.Subscriber('/tf',TFMessage,self.tf_callback,queue_size=2)
        rospy.Subscriber('/rgb/image_raw', Image, self.color_callback, )
        rospy.sleep(1)

    def tf_callback(self,msg):
        self.ps_msg = msg
        
        self.count=self.count+1


    def color_callback(self,image_msg):
        self.images = image_msg
        # image = self.cvbridge.imgmsg_to_cv2(image_msg,  desired_encoding='rgb8')
        
        # self.images = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
        # self.images = self.images[192:1344,550:1702]
        # self.images = cv2.resize(self.images,(256,256))
        
def move(next_pos, recordros):
    # a = MoveGroupTutorial()
    client = TrajectoryClient()
    print(next_pos)
    next_pos = np.array(next_pos).reshape(1,7)

    for i in range(1):
        # next_pos_ur = np.array(next_pos[i][:3])
        # next_quat_ur = np.array(next_pos[i][3:])
        # next_pos_moveit,next_quat_moveit =  get_new_pos(next_pos_ur, next_quat_ur)

        # a.go_to_pose_goal(np.concatenate((next_pos_moveit, next_quat_moveit)))
        
        client.send_cartesian_trajectory(next_pos[i])
        if len(recordros.obs_arr) == 0:
            recordros.obs_arr.append(recordros.images)
            recordros.obs_arr.append(recordros.images)
        else:
            recordros.obs_arr[1] = recordros.obs_arr[0]
            recordros.obs_arr[0] = recordros.images
        if len(recordros.state_arr) == 0:
            recordros.state_arr.append(recordros.ps_msg)
            recordros.state_arr.append(recordros.ps_msg)
        else:
            recordros.state_arr[1] = recordros.state_arr[0]
            recordros.state_arr[0] = recordros.ps_msg

def data_move(next_pos):

    client = TrajectoryClient()
    print(next_pos)


    for i in range(2):
        client.send_cartesian_trajectory(next_pos[i])


def add_two_ints_client(x, y):
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', ddt)
        # ipdb.set_trace()
        resp1 = add_two_ints(x, y)
        return resp1.next_state
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    rospy.init_node('kehu', anonymous = True)
    recordros = RecordROS() 
    rate = rospy.Rate(1)

    # data_path = '/home/yhx/yhx/catkin_ws/src/Universal_Robots_ROS_Driver/ur_robot_driver/scripts/data/wall/demo_60'
    # txt_path = '/home/yhx/yhx/catkin_ws/src/Universal_Robots_ROS_Driver/ur_robot_driver/scripts/data/wall/demo_60/tf_record.txt'
    # horizon = 0 
    # for filename in os.listdir(data_path):
    #     if filename.endswith('.png'):
    #         horizon += 1
    # with open(txt_path, 'r') as f:
    #         lines = f.readlines()
    
    # list_pos = []
    # count = 0
    # for j in range(1, horizon+1):
    #     count = count + 1
    #     # get image
    #     print(j)    
    #     image_path = data_path + '/image{}.png'.format(j)
    #     image = Image.open(image_path)
    #     temp = np.array(image) # 
    #     if len(temp) != 256:
    #         image = image.resize((256,256))
    #         # image.save("edge_resize/demo_{}_image{}.png".format(i,j))
    #     image = np.array(image)

    #     # get pose
    #     start = (j-1) * 16 + 9
    #     x = float(lines[start][7:])
    #     y = float(lines[start + 1][7:])
    #     z = float(lines[start + 2][7:])

    #     q1 = float(lines[start + 4][7:])
    #     q2 = float(lines[start + 5][7:])
    #     q3 = float(lines[start + 6][7:])
    #     q4 = lines[start + 7]
    #     end_q4 = q4.find("]")
    #     q4 = float(q4[7:end_q4])

    #     pos = np.array([x,y,z])
    #     quat = np.array([q1, q2, q3, q4])

    #     next_eef = np.concatenate((pos, quat))

    #     if len(list_pos) == 0:
    #         list_pos.append(next_eef)
    #         list_pos.append(next_eef)
    #     else:
    #         list_pos[0] = list_pos[1]
    #         list_pos[1] = next_eef
    #     if count % 2 == 0:
    #         data_move(list_pos)




    if len(recordros.obs_arr) == 0:
        recordros.obs_arr.append(recordros.images)
        recordros.obs_arr.append(recordros.images)
    else:
        recordros.obs_arr[1] = recordros.obs_arr[0]
        recordros.obs_arr[0] = recordros.images
    if len(recordros.state_arr) == 0:
        recordros.state_arr.append(recordros.ps_msg)
        recordros.state_arr.append(recordros.ps_msg)
    else:
        recordros.state_arr[1] = recordros.state_arr[0]
        recordros.state_arr[0] = recordros.ps_msg
        
    while not rospy.is_shutdown():
        obs = recordros.obs_arr
        state = recordros.state_arr
        # print(len(obs))
        # print(len(state))



        next_pos = add_two_ints_client(obs, state)
        move(next_pos, recordros)
        
        # print("Requesting %s+%s"%(obs, state))
        print(next_pos)
        rate.sleep()
    # a = MoveGroupTutorial()