# -*- coding: utf-8 -*-
import torch
import numpy as np
import transforms3d


class BaxterForwardKinematics(torch.nn.Module):
    # 已使用pykdl库使用UDRF文件验证,坐标系为URDF定义的坐标系(注意和Kinematics的word文档中的定义有所不同)
    def __init__(self):
        super(BaxterForwardKinematics, self).__init__()
        # L = np.array([270.35, 69.00, 364.35, 69.00, 374.29, 10.00, 229.525]) / 1000.
        # L = np.array([270.35, 69.00, 364.42, 69.00, 374.29, 10.00, 229.525]) / 1000.
        L = np.array([270.35, 69.00, 364.42, 69.00, 374.29, 10.00, 229.525 + 163]) / 1000.
        # 变换矩阵的sin部分
        self.mask_sin = torch.nn.Parameter(torch.LongTensor([
            [[0, -1, 0, 0],
             [1, 0, 0, 0],
             [0, 0, 0, 0],
             [0, 0, 0, 0]],
            [[-1, 0, 0, 0],
             [0, 0, 0, 0],
             [0, 1, 0, 0],
             [0, 0, 0, 0]],
            [[0, -1, 0, 0],
             [0, 0, 0, 0],
             [1, 0, 0, 0],
             [0, 0, 0, 0]],
            [[0, -1, 0, 0],
             [0, 0, 0, 0],
             [-1, 0, 0, 0],
             [0, 0, 0, 0]],
            [[0, -1, 0, 0],
             [0, 0, 0, 0],
             [1, 0, 0, 0],
             [0, 0, 0, 0]],
            [[0, -1, 0, 0],
             [0, 0, 0, 0],
             [-1, 0, 0, 0],
             [0, 0, 0, 0]],
            [[0, -1, 0, 0],
             [0, 0, 0, 0],
             [1, 0, 0, 0],
             [0, 0, 0, 0]]
        ]), requires_grad=False)
        # 变换矩阵的cos部分
        self.mask_cos = torch.nn.Parameter(torch.LongTensor([
            [[1, 0, 0, 0],
             [0, 1, 0, 0],
             [0, 0, 0, 0],
             [0, 0, 0, 0]],
            [[0, -1, 0, 0],
             [0, 0, 0, 0],
             [-1, 0, 0, 0],
             [0, 0, 0, 0]],
            [[1, 0, 0, 0],
             [0, 0, 0, 0],
             [0, 1, 0, 0],
             [0, 0, 0, 0]],
            [[1, 0, 0, 0],
             [0, 0, 0, 0],
             [0, -1, 0, 0],
             [0, 0, 0, 0]],
            [[1, 0, 0, 0],
             [0, 0, 0, 0],
             [0, 1, 0, 0],
             [0, 0, 0, 0]],
            [[1, 0, 0, 0],
             [0, 0, 0, 0],
             [0, -1, 0, 0],
             [0, 0, 0, 0]],
            [[1, 0, 0, 0],
             [0, 0, 0, 0],
             [0, 1, 0, 0],
             [0, 0, 0, 0]]
        ]), requires_grad=False)
        # 变换矩阵的固定部分
        self.offset = torch.nn.Parameter(torch.Tensor([
            [[0, 0, 0, 0],
             [0, 0, 0, 0],
             [0, 0, 1, 0],
             [0, 0, 0, 1]],
            [[0, 0, 0, L[1]],
             [0, 0, 1, 0],
             [0, 0, 0, 0],
             [0, 0, 0, 1]],
            [[0, 0, 0, 0],
             [0, 0, -1, -L[2]],
             [0, 0, 0, 0],
             [0, 0, 0, 1]],
            [[0, 0, 0, L[3]],
             [0, 0, 1, 0],
             [0, 0, 0, 0],
             [0, 0, 0, 1]],
            [[0, 0, 0, 0],
             [0, 0, -1, -L[4]],
             [0, 0, 0, 0],
             [0, 0, 0, 1]],
            [[0, 0, 0, L[5]],
             [0, 0, 1, 0],
             [0, 0, 0, 0],
             [0, 0, 0, 1]],
            [[0, 0, 0, 0],
             [0, 0, -1, 0],
             [0, 0, 0, 0],
             [0, 0, 0, 1]]
        ]), requires_grad=False)
        # 变换矩阵的开始部分和末端部分 从left/right_arm_mount开始 到left/right_hand为止
        self.As = torch.nn.Parameter(torch.Tensor([
            [1, 0, 0, 0.055695],
            [0, 1, 0, 0],
            [0, 0, 1, L[0]+0.011038],
            [0, 0, 0, 1]
        ]), requires_grad=False)
        self.Ae = torch.nn.Parameter(torch.Tensor([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, L[6]],
            [0, 0, 0, 1]
        ]), requires_grad=False)
        self.some_affines()
        
    def some_affines(self):
        # 这些参数来自URDF
        T = [0.024645, 0.219645, 0.118588]
        R = transforms3d.euler.euler2mat(0, 0, 0.7854, 'rxyz')
        Awl = transforms3d.affines.compose(T, R, np.ones([3]))
        self.Awl = torch.nn.Parameter(torch.Tensor(Awl), requires_grad=False)
        T = [0.024645, -0.219645, 0.118588]
        R = transforms3d.euler.euler2mat(0, 0, -0.7854, 'rxyz')
        Awr = transforms3d.affines.compose(T, R, np.ones([3]))
        self.Awr = torch.nn.Parameter(torch.Tensor(Awr), requires_grad=False)
        
    def forward(self, q, arm, detial=False):
        # 构造仿射矩阵
        if q.size(-1) == 6:
            q = torch.cat([q[..., :2], torch.zeros_like(q[..., :1]), q[..., 2:]], dim=-1)
        q = q[..., None, None]
        A = q.sin() * self.mask_sin + q.cos() * self.mask_cos + self.offset
        A = A.unbind(1)
        # 添加base
        if arm == "left":
            As = torch.einsum("ab,bc->ac", self.Awl, self.As)
            points = [self.Awl[:3, 3][None, ...], As[:3, 3][None, ...]]
        elif arm == "right":
            As = torch.einsum("ab,bc->ac", self.Awr, self.As)
            points = [self.Awr[:3, 3][None, ...], As[:3, 3][None, ...]]
        else:
            As = self.As
            points = [As[:3, 3][None, ...]]
        # 合并/分开计算
        if not detial:
            AA = torch.einsum("ab,nbc,ncd,nde,nef,nfg,ngh,nhi,ij->naj", As, A[0], A[1], A[2], A[3], A[4], A[5], A[6], self.Ae)
            return AA
        else:
            # base, left/right_mount, shoulder, shoulder, forearm, hand
            AA = torch.einsum("ab,nbc,ncd->nad", As, A[0], A[1])
            points.append(AA[:, :3, 3])
            AA = torch.einsum("nad,nde,nef->naf", AA, A[2], A[3])
            points.append(AA[:, :3, 3])
            AA = torch.einsum("naf,nfg,ngh,nhi->nai", AA, A[4], A[5], A[6])
            points.append(AA[:, :3, 3])
            AA = torch.einsum("nai,ij->naj", AA, self.Ae)
            points.append(AA[:, :3, 3])
            return AA, torch.stack(points, dim=1)
