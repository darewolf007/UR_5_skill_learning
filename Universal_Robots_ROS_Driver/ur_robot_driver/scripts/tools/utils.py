# -*- coding: utf-8 -*-
import torch
import transforms3d
import numpy as np


def affines_compose(T, R):
    """
    Compose T and R to affines. 组合平移和旋转矩阵为仿射矩阵。
    Expects two tensors of shape T(*, 3) and R(*, 3, 3), where * denotes any number of dimensions.
    Returns a tensor of shape A(*, 4, 4).
    """
    assert T.shape[-1] == 3
    assert R.shape[-1] == 3 and R.shape[-2] == 3
    original_shape = list(R.shape)
    original_shape[-2] = 4
    original_shape[-1] = 4

    T = T.reshape(-1, 3, 1)
    R = R.reshape(-1, 3, 3)
    A = torch.cat([R, T], dim=2)  # (-1, 3, 4)

    O = torch.zeros_like(A[:, :1])
    O[:, :, -1] = 1
    A = torch.cat([A, O], dim=1)
    return A.reshape(original_shape)


def axmtrix(r, epsilon=1e-9):
    """
    Convert axangle to RotMat. 轴角转换为旋转矩阵的pytorch实现
    Expects a tensor of shape (*, 3), where * denotes any number of dimensions.
    Returns a tensor of shape (*, 3, 3).
    """
    assert r.shape[-1] == 3
    original_shape = list(r.shape[:-1])
    original_shape = original_shape + [3, 3]
    r = r.reshape(-1, 3)

    n = r.shape[0]
    theta = r.norm(dim=1, keepdim=True) + epsilon
    xyz = r / theta
    theta = theta[..., None]

    x = xyz[:, 0]
    y = xyz[:, 1]
    z = xyz[:, 2]

    o = torch.zeros_like(x)
    M = torch.stack([o, -z, y, z, o, -x, -y, x, o], dim=1).reshape([-1, 3, 3])  # (N, 3, 3)
    M2 = torch.einsum("nij,njk->nik", M, M)
    R = torch.eye(3).reshape([-1, 3, 3]) + theta.sin() * M + (1 - theta.cos()) * M2
    R = R.view(original_shape)
    return R
