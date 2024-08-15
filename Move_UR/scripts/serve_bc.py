#! /home/yhx/miniforge3/envs/robodiff/bin/python

from __future__ import print_function

from Move_UR.srv import ddt,ddtResponse
import rospy
import yaml
import ipdb
import math
import numpy as np
from cv_bridge import CvBridge
import cv2
import torch

def quaternion_to_euler(q, degree_mode=0):
    qw, qx, qy, qz = q

    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx ** 2 + qy ** 2))
    pitch = math.asin(2 * (qw * qy - qz * qx))
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy ** 2 + qz ** 2))
    # degree_mode=1:【输出】是角度制，否则弧度制
    if degree_mode == 1:
        roll = np.rad2deg(roll)
        pitch = np.rad2deg(pitch)
        yaw = np.rad2deg(yaw)
    euler = np.array([roll, pitch, yaw])
    return euler
 
# 欧拉角转四元数
# ================OKOK
def euler_to_quaternion(euler, degree_mode=0):
    roll, pitch, yaw = euler
    # degree_mode=1:【输入】是角度制，否则弧度制
    if degree_mode == 1:
        roll = np.deg2rad(roll)
        pitch = np.deg2rad(pitch)
        yaw = np.deg2rad(yaw)

    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    q = np.array([qw, qx, qy, qz])
    return q

def load_yaml_file(file_path):
    with open(file_path, "r") as file:
        try:
            data = yaml.safe_load(file)
            return data
        except yaml.YAMLError as e:
            return None

def handle_add_two_ints(req):
    import numpy as np
    import os
    import sys
    import torch
    import hydra
    sys.path.append('/home/yhx/decision-transformer/gym/decision_transformer/training')
    sys.path.append('/home/yhx/decision-transformer/gym/decision_transformer/models')
    sys.path.append('/home/yhx/decision-transformer/gym/')
    



    checkpoint = '/home/yhx/bc/open_box/model-90.pt' # hunhe
    model = torch.load(checkpoint, map_location="cuda:0")
    device = "cuda:0"
    model.to(device)
    model.eval()

    # device = 'cuda:0'
    # device = torch.device(device)
    # policy.to(device)
    # policy.eval()

    # #实例化policy
    # yaml_file_path = "/home/yhx/diffusion_policy/my_image_ph_diffusion_policy_cnn.yaml"
    # yaml_data =  load_yaml_file(yaml_file_path)
    # obs_input_dict = {}
    # ipdb.set_trace()
    # image_1 = cv2.imread(req.obs[0].data)
    # # print(image_1.shape)
    # image_1 = image_1[192:1344,550:1702]
    # image_1 = cv2.resize(image_1,(256,256))

    import ros_numpy
    # print(len(req.obs))
    # images = req.obs[0]
    # images = ros_numpy.numpify(images)
    # images = np.ascontiguousarray(images[:,:,:3])  
    # # images = cv2.cvtColor(np.uint8(images), cv2.COLOR_GRAY2BGR)
    # # images = images[192:1344,550:1702] #push to wall 
    # images = images[92:1344,350:1902] # occlusion
    
    # images = cv2.resize(images,(256,256))
    # # print(images.shape)
    # image_1 = np.transpose(images, (2,0,1))
    # # print(image_1.shape)
    # imagefile = '/home/yhx/yhx/cv/image.png'
    # cv2.imwrite(imagefile, images)
    # image_1 = np.zeros((3,256,256))
    # ipdb.set_trace()
    state_x = req.state[0].transforms[0].transform.translation.x 
    state_y = req.state[0].transforms[0].transform.translation.y
    state_z = req.state[0].transforms[0].transform.translation.z 

    state_q1 = req.state[0].transforms[0].transform.rotation.x 
    state_q2 = req.state[0].transforms[0].transform.rotation.y 
    state_q3 = req.state[0].transforms[0].transform.rotation.z 
    state_q4 = req.state[0].transforms[0].transform.rotation.w 

    eef_pos = np.array([state_x, state_y, state_z])

    eef_quat = np.array([state_q1, state_q2, state_q3, state_q4])

    eef = np.concatenate([eef_pos, eef_quat], axis=0)
    state = torch.tensor(eef).unsqueeze(dim=0).unsqueeze(dim=0)

    action = torch.zeros(6).unsqueeze(dim=0).unsqueeze(dim=0)
    reward = torch.tensor([0])

    with torch.no_grad():
        a = model.get_action(states=state.to(device), actions = action.to(device), rewards=reward.to(device))
    # a = policy.predict_action(obs_input_dict)
    print("action",a)

    current_pos = eef_pos
    current_rpy = quaternion_to_euler(eef_quat)

    future_eef = []

    woyao_a = np.array(a.detach().cpu())
    delta_pos = woyao_a[:3]
    delta_rpy = woyao_a[3:]
        
        # if current_pos[1] > -0.12 and current_pos[2] > 0.05:
        #     delta_rpy = np.array([0, 0, 0])
        #     current_pos[2] = current_pos[2] - 0.035
        #     current_pos[1] = current_pos[1] + 0.005
        
        # if current_pos[1] > 0:
        #     current_rpy = current_rpy + delta_rpy
        # else:
        #     current_rpy = current_rpy
        # delta_rpy[0] = 0
        # delta_rpy[2] = 0
    current_pos = current_pos + delta_pos
    current_rpy = current_rpy + delta_rpy

    current_quat = euler_to_quaternion(current_rpy)
        # if current_pos[2] < 0.05 and current_pos[1] > 0.02:
        #     current_quat = np.array([0.500000000, 0.500000000, -0.500000000, 0.500000000])
        #     current_pos[1] = current_pos[1] - 0.02

        # else:
        #     current_quat = euler_to_quaternion(current_rpy)
    assert current_pos[0] > 0
        
    print("delta_pos:", delta_pos)
    print("current_pos:", current_pos)
    print("delta_quat:", delta_rpy)
    print("current_quat:", current_quat)
    print("=========================================")
        

    future_eef = np.concatenate((current_pos, current_quat))
        # future_eef = np.concatenate((current_pos, euler_to_quaternion(current_rpy))).astype(np.float64)
    # ipdb.set_trace()
    next_eef = future_eef


    b = [next_eef]

    print(next_eef)
    return b

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    obs = rospy.Service('add_two_ints', ddt, handle_add_two_ints)
    
    print("Ready to add two ints.")
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()