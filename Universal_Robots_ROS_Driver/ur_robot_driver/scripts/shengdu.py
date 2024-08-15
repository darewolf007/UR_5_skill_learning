import cv2
import os.path
import glob
import numpy as np
from PIL import Image


pi = np.pi
def quaternion_to_rot(q): 
    q0,q1,q2,q3=q 

    R=np.array([[1-2*(q2**2+q3**2),2*(q1*q2-q3*q0),2*(q1*q3+q2*q0)], 
                     [2*(q1*q2+q3*q0),1-2*(q1**2+q3**2),2*(q2*q3-q1*q0)], 
                     [2*(q1*q3-q2*q0),2*(q2*q3+q1*q0),1-2*(q1**2+q2**2)]]) 
    return R 

def transform_uv_to_xy(R, T, K, uv, depth):
    xy_reshape = np.array(uv).reshape((3,1))
    T_reshape = np.array(T).reshape((3,1))
    K_inv = np.linalg.inv(K)

    print(K_inv)
    print(depth)
    print('--------------------------------')

    T_diff = depth*np.dot(K_inv, xy_reshape)
    
    world_xyz = np.dot(R, T_diff)+ T_reshape

    #s = np.einsum("ab,nb->na", color_R, xy_reshape) + color_T
    #world_xyz = np.einsum("ab,nb->na", R, xy_reshape) + T
    return world_xyz.flatten()



depthfile = '/home/xj/Documents/catkin_ws/src/Universal_Robots_ROS_Driver/ur_robot_driver/scripts/depth/depth.png'
img = cv2.imread(depthfile, flags = cv2.IMREAD_UNCHANGED)
# Q = np.array([0.04629467912385805,-0.0025500403239896653,-0.02599180286203684,-0.9985863638980588])
Q = np.array([-0.0025500403239896653,-0.02599180286203684,-0.9985863638980588,0.04629467912385805])
# Q = np.array([0.00737717708335,-0.0234853715093,-0.756037359856,0.654065382883])
R = quaternion_to_rot(Q)
print(R)
robotiq_pos = [-264.60,-451.50,405.75]
robotiq_euler = []
T0 = T_cam2end = [28.679352547256553,83.8954421203303,(200-63.035287708976455)]
print("T0",T0)
T1 = [T0[0], T0[2]*np.cos(pi/3)-T0[1]*np.sin(pi/3), T0[2]*np.sin(pi/3)+T0[1]*np.cos(pi/3)]
print("T1",T1)
T = [(T1[0]-T1[1])*np.cos(pi/4)+T1[1]/np.cos(pi/4)+robotiq_pos[0], robotiq_pos[1]-(T1[0]-T1[1])*np.cos(pi/4),T1[2]+robotiq_pos[2]]
print("T",T)
K = np.array([920.5091552734375, 0.0, 643.0638427734375, 0.0, 921.207275390625, 349.23675537109375, 0.0, 0.0, 1.0]).reshape([3,3])
uv = [543,306,1]
depth = img[uv[1],uv[0]]
xyz = transform_uv_to_xy(R,T,K,uv,depth)
print(xyz)



