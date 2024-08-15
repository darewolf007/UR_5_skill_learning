# -*- coding: utf-8 -*-
import numpy as np
import torch

from baxter_interface import CHECK_VERSION
import baxter_interface
import rospy

from tools.pid import PID
from tools.fk import BaxterForwardKinematics


class IOBaxterControl(object):
    """控制接口"""
    def __init__(self, k=[3, -0.3, +30]):
        self.left_arm = baxter_interface.limb.Limb("left")
        self.left_joint_names = self.left_arm.joint_names()
        self.right_arm = baxter_interface.limb.Limb("right")
        self.right_joint_names = self.right_arm.joint_names()
        # self.pid = PID(k=k, limit=[0.7, 0.7, 1., 1., 2., 2., 2.])
        self.pid = [
            # PID(k=k, limit=[0.2, 0.2, 0.4, 0.4, 0.6, 0.6, 0.6]),
            # PID(k=k, limit=[0.2, 0.2, 0.4, 0.4, 0.6, 0.6, 0.6])]
            PID(k=k, limit=[0.4, 0.4, 0.6, 0.6, 0.6, 0.6, 0.6]),
            PID(k=k, limit=[0.4, 0.4, 0.6, 0.6, 0.6, 0.6, 0.6])]
        self.fk = BaxterForwardKinematics()
        print("Enabling robot... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        self._rs.enable()

    def joint_angles(self, arm='left'):
        if arm == 'left':
            _angles = self.left_arm.joint_angles()
            js = np.array([_angles[k] for k in self.left_joint_names])
        elif arm == 'right':
            _angles = self.right_arm.joint_angles()
            js = np.array([_angles[k] for k in self.right_joint_names])
        return js

    def endpoint_pose(self, arm='left'):
        q = self.joint_angles(arm)
        with torch.no_grad():
            A = self.fk(torch.Tensor([q]), arm).numpy()
        return A

    def __call__(self, target):
        # target_traj [2(left, right), t, 7] 
        # 获取机械臂关节当前位置->当前位置和目标位置送进PID计算速度->速度控制
        # 左臂 计算PID与控制
        for i in range(target.shape[0]):
            if i == 0:
                v = self.pid[i](self.joint_angles('left'), target[i])
                cmd = {k: v[ii] for ii, k in enumerate(self.left_joint_names)}
                self.left_arm.set_joint_velocities(cmd)
            else:
                v = self.pid[i](self.joint_angles('right'), target[i])
                cmd = {k: v[ii] for ii, k in enumerate(self.right_joint_names)}
                self.right_arm.set_joint_velocities(cmd)
            
    def do_traj(self, target_traj):
        # target_traj [2(left, right), t, 7] 
        # 获取机械臂关节当前位置->当前位置和目标位置送进PID计算速度->速度控制
        # 左臂 计算PID与控制
        v = self.pid(self.joint_angles('left'), target_traj[0])
        cmd = {k: v[i] for i, k in enumerate(self.left_joint_names)}
        self.left_arm.set_joint_velocities(cmd)

    def clean(self):
        # 机械臂速度=0
        cmd = {k: 0. for i, k in enumerate(self.left_joint_names)}
        self.left_arm.set_joint_velocities(cmd)
        rospy.sleep(1.)
