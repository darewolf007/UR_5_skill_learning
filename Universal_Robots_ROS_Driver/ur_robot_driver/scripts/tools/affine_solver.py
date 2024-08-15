# -*- coding: utf-8 -*-
from tools.utils import axmtrix
from tools.container import AffineContainer
import torch
import numpy as np


def affine_solver(p_camera, p_robot, max_iter=10001, eps=1e-5, device="cpu", print_progress=False):
    """
    求解外参的IO函数，输入为点在相机坐标系下的坐标和其对应机器人（世界）坐标系下的坐标，返回仿射矩阵。
    """
    print(p_camera.shape, p_robot.shape)

    net = AffineContainer().to(device)
    opt = torch.optim.Adam(net.parameters(), lr=0.05)
    x, y = p_camera, p_robot

    n = x.shape[0]
    for i in range(max_iter):
        net.zero_grad()
        y_ = net(x)
        loss = ((y - y_) ** 2).mean()
        loss.backward()
        opt.step()
        if i % 100 == 0:
            loss = loss.item()
            if print_progress:
                print(i, loss)
            if loss < eps:
                break

    T = net.T.detach().cpu().numpy()
    R = axmtrix(net.r.detach().cpu()).numpy()
    return (T, R)


if __name__ == "__main__":
    # 简单的测试
    T = torch.randn(3)
    r = torch.randn(3)
    R = axmtrix(r)
    x = torch.randn(100, 3)
    y = torch.einsum("ij,nj->ni", R, x + torch.randn_like(x) * 1e-2) + T[None, :]

    print(T.numpy(), R.numpy())
    print(affine_solver(x, y, print_progress=True))
