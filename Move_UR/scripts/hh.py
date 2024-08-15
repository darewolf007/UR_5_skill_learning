import numpy as np
import math


def get_new_pos(O_eef,q):
    old_q = q.copy()

    temp = q[3]
    q[3] = q[2]
    q[2] = q[1]
    q[1] = q[0]
    q[0] = temp

    q0,q1,q2,q3=q 

    R=np.array([[1-2*(q2**2+q3**2),2*(q1*q2-q3*q0),2*(q1*q3+q2*q0)], 
                     [2*(q1*q2+q3*q0),1-2*(q1**2+q3**2),2*(q2*q3-q1*q0)], 
                     [2*(q1*q3-q2*q0),2*(q2*q3+q1*q0),1-2*(q1**2+q2**2)]]) 
    

    pos_eef = np.array([0,0, -0.23])
    x_new = R[0][0] * pos_eef[0] + R[0][1] * pos_eef[1] + R[0][2] * pos_eef[2] + O_eef[0]
    y_new = R[1][0] * pos_eef[0] + R[1][1] * pos_eef[1] + R[1][2] * pos_eef[2] + O_eef[1]
    z_new = R[2][0] * pos_eef[0] + R[2][1] * pos_eef[1] + R[2][2] * pos_eef[2] + O_eef[2]
    pos_new = np.array([-x_new, -y_new, z_new])

    old_q0 = old_q[0]
    old_q1 = old_q[1]
    old_q2 = old_q[2]
    old_q3 = old_q[3]

    new_q = np.array([old_q1, -old_q0, -old_q3, old_q2])


    print("pos_new:",pos_new)
    print("quat_new:", new_q)
    return pos_new, new_q

if __name__ == "__main__":

    yuan_pos = np.array([0.645,-0.098,0.288])
    yuan_quat = np.array([0.036, -0.974, 0.199,0.101])







    get_new_pos( yuan_pos,yuan_quat) 

