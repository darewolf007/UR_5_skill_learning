# -*- coding: utf-8 -*-
from tools.utils import axmtrix
import torch


class AffineContainer(torch.nn.Module):
    """注意这里的仿真矩阵均不考虑缩放，因此完全等价于旋转矩阵和平移向量的组合。"""
    def __init__(self):
        super(AffineContainer, self).__init__()
        self.T = torch.nn.Parameter(torch.zeros([3]))
        self.r = torch.nn.Parameter(torch.zeros([3]))

    def forward(self, p_camrea):
        original_shape = list(p_camrea.shape)
        p_camrea = p_camrea.reshape([-1, 3])
        p_robot = torch.einsum("ij,nj->ni", axmtrix(self.r), p_camrea) + self.T
        return p_robot.reshape(original_shape)


class BaxterQContainer(torch.nn.Module):
    def __init__(self, q=torch.zeros([1, 7])):
        super(BaxterQContainer, self).__init__()
        self.q = torch.nn.Parameter(q.clone())

    def forward(self):
        return self.q

class PointContainer(torch.nn.Module):
    def __init__(self, p=torch.zeros([1, 3])):
        super(PointContainer, self).__init__()
        self.p = torch.nn.Parameter(p.clone())

    def forward(self):
        return self.p
