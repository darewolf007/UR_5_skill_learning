# -*- coding: utf-8 -*-
import numpy as np


class PID(object):
    def __init__(self, k, limit=None):
        # k = [3, 0.01, 30]
        # limit = [0.7, 0.7, 1., 1., 2., 2., 2.]
        self.k = k
        self.limit = limit
        self.e_sum = np.zeros([len(limit)])
        self.d_last = np.zeros([len(limit)])
        # self.c_last = np.zeros([len(limit)])

    def __call__(self, current, target):
        assert current.shape == target.shape
        # 计算误差
        err = target - current
        # 计算控制量
        c = self.k[0] * err + self.k[1] * self.e_sum + self.k[2] * (self.d_last - current)
        # c = c * 0.5 + self.c_last * 0.5
        c = [min(c[i], self.limit[i]) for i in range(len(self.limit))]
        c = [max(c[i], -self.limit[i]) for i in range(len(self.limit))]
        # update
        self.e_sum = err + self.e_sum * 0.5
        self.d_last = current
        self.c_last = np.asarray(c)
        return c

