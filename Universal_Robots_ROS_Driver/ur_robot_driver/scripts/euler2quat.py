#!/usr/bin/python
# -*- coding: utf-8 -* 
import math
import numpy as np 

# from pydrake.all import RotationMatrix, RollPitchYaw
 
# 四元数转欧拉角
# ================OKOK
def quaternion_to_euler(q, degree_mode=1):
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
# def euler_to_quaternion(euler, degree_mode=1):
#     yaw, pitch, roll = euler
#     # degree_mode=1:【输入】是角度制，否则弧度制
#     if degree_mode == 1:
#         roll = np.deg2rad(roll)
#         pitch = np.deg2rad(pitch)
#         yaw = np.deg2rad(yaw)

#     qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
#     qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
#     qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
#     qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
#     q = np.array([qx, qy, qz, qw])
#     return q
def euler_to_quaternion(euler, degree_mode=1):
    pitch, yaw, roll = euler
    # degree_mode=1:【输入】是角度制，否则弧度制
    if degree_mode == 1:
        roll = np.deg2rad(roll)
        pitch = np.deg2rad(pitch)
        yaw = np.deg2rad(yaw)

    qx = np.sin(pitch/2) * np.cos(yaw/2) * np.cos(roll/2) - np.cos(pitch/2) * np.sin(yaw/2) * np.sin(roll/2)
    qy = np.cos(pitch/2) * np.sin(yaw/2) * np.cos(roll/2) + np.sin(pitch/2) * np.cos(yaw/2) * np.sin(roll/2)
    qz = np.cos(pitch/2) * np.cos(yaw/2) * np.sin(roll/2) - np.sin(pitch/2) * np.sin(yaw/2) * np.cos(roll/2)
    qw = np.cos(pitch/2) * np.cos(yaw/2) * np.cos(roll/2) + np.sin(pitch/2) * np.sin(yaw/2) * np.sin(roll/2)
    q = np.array([qw, qx, qy, qz])
    return q

# 四元数转旋转矩阵：
def quaternion_to_rot(q): 
    q0,q1,q2,q3=q 

    R=np.array([[1-2*(q2**2+q3**2),2*(q1*q2-q3*q0),2*(q1*q3+q2*q0)], 
                     [2*(q1*q2+q3*q0),1-2*(q1**2+q3**2),2*(q2*q3-q1*q0)], 
                     [2*(q1*q3-q2*q0),2*(q2*q3+q1*q0),1-2*(q1**2+q2**2)]]) 
    return R 
 
# 旋转矩阵转四元数：
# ================OKOK
def rot_to_quaternion(R): 
    R=R 
    
    qw=np.sqrt(1+R[0,0]+R[1,1]+R[2,2])/2 
    qx=(R[2,1]-R[1,2])/(4*qw) 
    qy=(R[0,2]-R[2,0])/(4*qw) 
    qz=(R[1,0]-R[0,1])/(4*qw) 
    q = np.array([qw, qx, qy, qz])
    return q

# 旋转矩阵转欧拉角
# ================OKOK
def rot_to_euler(R, degree_mode=1):
    sy = np.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
 
    singular = sy < 1e-6
 
    if not singular :
        roll = np.arctan2(R[2,1] , R[2,2])
        pitch = np.arctan2(-R[2,0], sy)
        yaw = np.arctan2(R[1,0], R[0,0])
    else :
        roll = np.arctan2(-R[1,2], R[1,1])
        pitch = np.arctan2(-R[2,0], sy)
        yaw = 0
    
    # degree_mode=1:【输出】是角度制，否则弧度制
    if degree_mode == 1:
        roll = np.rad2deg(roll)
        pitch = np.rad2deg(pitch)
        yaw = np.rad2deg(yaw)

    euler = np.array([roll, pitch, yaw])
    return euler

# 欧拉角转旋转矩阵
# # ================OKOK
def euler_to_rot(euler, degree_mode=1):
    roll, pitch, yaw = euler
    # degree_mode=1:【输入】是角度制，否则弧度制
    if degree_mode == 1:
        roll = np.deg2rad(roll)
        pitch = np.deg2rad(pitch)
        yaw = np.deg2rad(yaw)

    R_x = np.array([
    [1,     0,              0              ],
    [0,     math.cos(roll), -math.sin(roll)],
    [0,     math.sin(roll), math.cos(roll) ]
    ])
 
    R_y = np.array([
    [math.cos(pitch),  0,   math.sin(pitch) ],
    [0,                1,   0               ],
    [-math.sin(pitch), 0,   math.cos(pitch) ]
    ])
 
    R_z = np.array([
    [math.cos(yaw), -math.sin(yaw),  0],
    [math.sin(yaw), math.cos(yaw),   0],
    [0,             0,               1]
    ])
    
    R = np.dot(R_z, np.dot( R_y, R_x ))
    return R


if __name__ == "__main__":
    # 测试思路：随机生成几组欧拉角，用各种转换后看是否与原数据相同
    # 备注：np.sin(),math.sin()的返回值都是弧度制
    # np.atan2(), math.asin()这些也是弧度制
    # print(np.sin(90), np.sin(45))
    # print(np.sin(3.14), np.sin(1.57))
    # print(math.sin(90), math.sin(45))
    # print(math.sin(3.14), math.sin(1.57))
    # mode = 0  # 当这个值等于1时, 输入和输出的欧拉角均为 <角度制>
    # for i in range(10):
    #     print('=' * 32, "index:", i, '=' * 35)
    #     euler = np.random.randint(0, 360, 3) - 180
    #     # euler = np.array([60, 30, 45])

    #     if mode != 1:
    #         # 限制最多小数点后三位
    #         euler = np.array([round(t * math.pi / 180, 3) for t in euler])

    #     print("euler = ", euler)
    #     e2q2e = quaternion_to_euler(euler_to_quaternion(euler, mode), mode)
    #     e2r2e = rot_to_euler(euler_to_rot(euler, mode), mode)
    #     e2r2q2e = quaternion_to_euler(rot_to_quaternion(euler_to_rot(euler, mode)), mode)
    #     e2q2r2e = rot_to_euler(quaternion_to_rot(euler_to_quaternion(euler, mode)), mode)
        
    #     # 限制这些值为[-180,179)
    #     if mode == 1:
    #         e2q2e = [(int((180 * 3 + t)) % 360) - 180 for t in e2q2e]
    #         e2r2e = [(int((180 * 3 + t)) % 360) - 180 for t in e2r2e]
    #         e2r2q2e = [(int((180 * 3 + t)) % 360) - 180 for t in e2r2q2e]
    #         e2q2r2e = [(int((180 * 3 + t)) % 360) - 180 for t in e2q2r2e]
    #     else:  # 限制最多小数点后三位
    #         e2q2e = [round((int((math.pi * 3 * 1000 + t * 1000)) % 6283 - 3142) / 1000, 3)\
    #              for t in e2q2e]    
    #         e2r2e = [round((int((math.pi * 3 * 1000 + t * 1000)) % 6283 - 3142) / 1000, 3)\
    #              for t in e2r2e]   
    #         e2r2q2e = [round((int((math.pi * 3 * 1000 + t * 1000)) % 6283 - 3142) / 1000, 3)\
    #              for t in e2r2q2e]   
    #         e2q2r2e = [round((int((math.pi * 3 * 1000 + t * 1000)) % 6283 - 3142) / 1000, 3)\
    #              for t in e2q2r2e]     

    #     print("e -> q -> e = ", e2q2e, "norm of delta = ", np.linalg.norm(e2q2e - euler))
    #     print("e -> r -> e = ", e2r2e, "norm of delta = ", np.linalg.norm(e2r2e - euler))
    #     print("e -> r -> q -> e = ", e2r2q2e, "norm of delta = ", np.linalg.norm(e2r2q2e - euler))
    #     print("e -> q -> r -> e = ", e2q2r2e, "norm of delta = ", np.linalg.norm(e2q2r2e - euler))

        # if mode == 0:   
        #     # pydrake中的旋转角是弧度制
        #     print("drake actual rot = ", RotationMatrix(RollPitchYaw(euler[0],euler[1],euler[2])))
        #     print("rot =", euler_to_rot(euler, mode))
        #     print("e -> q -> rot =", quaternion_to_rot(euler_to_quaternion(euler, mode)))

    # eul = [180, 0, 90]
    quat = [0,-1,0,0]
    # quat = euler_to_quaternion(eul, degree_mode=1)
    eul = quaternion_to_euler(quat, degree_mode=1)

    print(quat)
    print(eul)

