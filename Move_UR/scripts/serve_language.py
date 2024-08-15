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
    sys.path.append( '/home/yhx/diffusion_policy/diffusion_policy/workspace')
    sys.path.append( '/home/yhx/diffusion_policy/diffusion_policy/policy')
    sys.path.append('/home/yhx/diffusion_policy/diffusion_policy/dataset')
    sys.path.append('/home/yhx/diffusion_policy')
    from my_diffusion_unet_hybrid_image_policy_language import  MyDiffusionUnetHybridImagePolicy
    from base_dataset import BaseImageDataset
    from base_workspace import BaseWorkspace
    import dill





    checkpoint = '/home/yhx/diffusion_policy/data/gouxing_1/latest.ckpt' # hunhe
    payload = torch.load(open(checkpoint, 'rb'), pickle_module=dill)
    cfg = payload['cfg']
    cls = hydra.utils.get_class(cfg._target_)
    workspace = cls(cfg)
    workspace: BaseWorkspace 
    workspace.load_payload(payload, exclude_keys=None, include_keys=None)

    policy = workspace.model

    # device = 'cuda:0'
    # device = torch.device(device)
    # policy.to(device)
    policy.eval()

    # #实例化policy
    yaml_file_path = "/home/yhx/diffusion_policy/my_image_ph_diffusion_policy_cnn_language.yaml"
    yaml_data =  load_yaml_file(yaml_file_path)
    obs_input_dict = {}
    # ipdb.set_trace()
    # image_1 = cv2.imread(req.obs[0].data)
    # # print(image_1.shape)
    # image_1 = image_1[192:1344,550:1702]
    # image_1 = cv2.resize(image_1,(256,256))

    import ros_numpy
    print(len(req.obs))
    images = req.obs[0]
    images = ros_numpy.numpify(images)
    images = np.ascontiguousarray(images[:,:,:3])  
    # images = cv2.cvtColor(np.uint8(images), cv2.COLOR_GRAY2BGR)
    images = images[92:1344,350:1902]
    images = cv2.resize(images,(256,256))
    # print(images.shape)
    image_1 = np.transpose(images, (2,0,1))
    # print(image_1.shape)
    imagefile = '/home/yhx/yhx/cv/image.png'
    cv2.imwrite(imagefile, images)
    # image_1 = np.zeros((3,256,256))
    # ipdb.set_trace()
    state_x = req.state[0].transforms[0].transform.translation.x 
    state_y = req.state[0].transforms[0].transform.translation.y
    state_z = req.state[0].transforms[0].transform.translation.z 

    state_q1 = req.state[0].transforms[0].transform.rotation.x 
    state_q2 = req.state[0].transforms[0].transform.rotation.y 
    state_q3 = req.state[0].transforms[0].transform.rotation.z 
    state_q4 = req.state[0].transforms[0].transform.rotation.w 

    eef_pos_1 = np.array([state_x, state_y, state_z])

    eef_quat_1 = np.array([state_q1, state_q2, state_q3, state_q4])

    eef_pos = []
    eef_quat = []
    agentview_image = []

    for i in range(2):
        eef_pos.append(eef_pos_1)
        eef_quat.append(eef_quat_1)
        agentview_image.append(image_1)
    
    eef_pos = np.array(eef_pos)[np.newaxis, ...]
    eef_quat = np.array(eef_quat)[np.newaxis, ...]
    agentview_image = np.array(agentview_image)[np.newaxis, ...]


    obs_input_dict['robot0_eef_pos'] = eef_pos
    obs_input_dict['robot0_eef_quat'] = eef_quat
    obs_input_dict['agentview_image'] = agentview_image

    print(eef_pos.shape)
    print(eef_quat.shape)
    print(agentview_image.shape)



    word = np.array([[4,4]])
    a = policy.predict_action(obs_input_dict, word)
    # print(a)

    current_pos = eef_pos_1
    current_rpy = quaternion_to_euler(eef_quat_1)

    future_eef = []
    for i in range(len(a['action'][0])):
    # for i in range(1):
        woyao_a = np.array(a['action'][0][i].detach())
        delta_pos = woyao_a[:3]
        delta_rpy = woyao_a[3:]
        # if current_pos[2] > 0.25:
        #     delta_pos[0] -= 0.02
        #     delta_pos[2] -= 0.01
        # else:
        #     delta_pos[1] -= 0.01
        
        # if current_pos[2] > 0.3:
        #     delta_pos[1] += 0.008
        # else:
        #     delta_pos[1] -= 0.003

        if delta_pos[0] < 0 and delta_pos[2] < 0.005:
            delta_pos[0] =delta_pos[0] * 1.2

        # if current_pos[2] < 0.3:
        #     delta_pos[2] -= 0.01
        # if current_pos[2] < 0.208:
        #     delta_pos[2] = 0
        #     delta_pos[1] -= 0.02
        # if current_pos[2] < 0.27:
        #     delta_pos[2] = 0
        #     delta_pos[1] -= 0.02
        
        # # pot weitiao
        # if current_pos[2] > 0.29:
        #     temp_0 = delta_pos[0]
        #     temp_1 = delta_pos[1]
        #     delta_pos[0] = temp_0 * 1.5
        #     delta_pos[1] = temp_1 * 2.5
        
        # if current_pos[1] > -0.12 and current_pos[2] > 0.05:
        #     delta_rpy = np.array([0, 0, 0])
        #     current_pos[2] = current_pos[2] - 0.035
        #     current_pos[1] = current_pos[1] + 0.005
        
        # if current_pos[1] > 0:
        #     current_rpy = current_rpy + delta_rpy
        # else:
        #     current_rpy = current_rpy
        
        # delta_rpy[0] = 0
        # delta_rpy[1] = 0
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
        

        future_eef.append(np.concatenate((current_pos, current_quat)))
        # future_eef = np.concatenate((current_pos, euler_to_quaternion(current_rpy))).astype(np.float64)
    # ipdb.set_trace()
    next_eef = []
    for i in range(2):
        for j in range(7):
            next_eef.append(future_eef[i][j])


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