# -*- coding: utf-8 -*-
from utils import KinectDK

import numpy as np
import scipy.io as scio
import cv2
import time

from baxter_interface import CHECK_VERSION
import rospy
import baxter_interface
import transforms3d


links = [(0, 1), (1, 2), (2, 3), (2, 4), (4, 5), (5, 6), (6, 7),
         (7, 8), (8, 9), (7, 10), (2, 11), (11, 12), (12, 13), (13, 14),
         (14, 15), (15, 16), (14, 17), (0, 18), (18, 19), (19, 20),
         (20, 21), (0, 22), (22, 23), (23, 24), (24, 25), (3, 26),
         (26, 27), (26, 28), (26, 29), (26, 30), (26, 31)
]
color_R = np.array([[0.999985, 0.00392864, -0.00372753],
                    [-0.00352813, 0.994782, 0.10196],
                    [0.00410865, -0.101946, 0.994781]])
color_T = np.array([-32.0521, -1.90638, 3.84897]) * 1e-3


def collection_human_trace():
    rospy.init_node('collection_human_trace')

    kinect_dk = KinectDK()
    img_color = kinect_dk.queue_color.get(timeout=10.0)
    K = np.asarray(kinect_dk.rgb_info.K).reshape(3, 3)
    D = np.asarray(kinect_dk.rgb_info.D)
    size = img_color.shape[:2][::-1]
    map1, map2 = cv2.initUndistortRectifyMap(K, D, None, None, size, cv2.CV_32FC1)

    color_map = {}
    state = False
    print("state: {}".format(state))

    human_id = None
    trace_t = []
    trace_r = []
    frame = 0
    while 1:
        # 获取矫正图片
        img_color = kinect_dk.queue_color.get(timeout=10.0)
        markers = kinect_dk.queue_hbody.get()
        # img_depth = kinect_dk.queue_depth.get(timeout=10.0)
        img_color = cv2.remap(img_color, map1, map2, cv2.INTER_CUBIC)
        # img_depth = cv2.remap(img_depth, map1, map2, cv2.INTER_NEAREST)
        img_color_disp = img_color.copy()
        font = cv2.FONT_HERSHEY_SIMPLEX

        # 1人以上
        if len(markers) > 0:
            # 选定主ID
            if human_id is None or len(markers) == 32:
                human_id_new = markers[0].id // 100
                if human_id_new != human_id:
                    print("[w] id change: {} -> {}".format(human_id, human_id_new))
                    human_id = human_id_new

            # 采集
            if state:
                frame += 1
                if frame % 150 == 0:
                    print("frame: {}({}s)".format(frame, frame / 15))
                for m in markers:
                    if (m.id // 100) == human_id:
                        t = (m.pose.position.x, m.pose.position.y, m.pose.position.z)
                        r = (m.pose.orientation.w, m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z)
                        trace_t.append(t)
                        trace_r.append(r)

            # 显示
            n = len(markers) // 32
            for h in range(n):
                cid = markers[32*h].id // 100
                if cid not in color_map:
                    color_map[cid] = tuple([np.random.randint(255) for _ in range(3)])

                # 关节点 从depth相机坐标系转换到color相机坐标系 单位：m
                p_marker_3d = np.array([(markers[i + h*32].pose.position.x, markers[i + h*32].pose.position.y, markers[i + h*32].pose.position.z) for i in range(32)])
                p_marker_3d = np.einsum("ij,nj->ni", color_R, p_marker_3d) + color_T

                # 关节点 转换到color坐标系的理想平面
                p_marker_idea = p_marker_3d / p_marker_3d[:, 2:]

                # 关节点 转换到像素平面
                p_marker_pixel = np.einsum("ij,nj->ni", K, p_marker_idea)
                uv_marker_pixel = p_marker_pixel.astype(int)[:, :2]  # 32x2

                for i in range(32):
                    img_color_disp = cv2.circle(img_color_disp, (uv_marker_pixel[i, 0], uv_marker_pixel[i, 1]), 6, color_map[cid], 10)
                for (i, j) in links:
                    img_color_disp = cv2.line(img_color_disp, (uv_marker_pixel[i, 0], uv_marker_pixel[i, 1]), (uv_marker_pixel[j,0], uv_marker_pixel[j,1]), color_map[cid], 3)
                cv2.putText(img_color_disp, "{}".format(cid), (uv_marker_pixel[27, 0], uv_marker_pixel[27, 1]), font, 1.5, (255, 255, 255), 3)

        img_color_disp = cv2.resize(img_color_disp, tuple(np.asarray(size)//2))
        cv2.imshow("image", img_color_disp)

        # 按键
        key = cv2.waitKey(1)
        if key == ord('p'):
            state = not state
            print("state: {}".format(state))
        elif key == ord('q'):
            break
    cv2.destroyAllWindows()
    kinect_dk.release()

    trace_t = np.asarray(trace_t)
    trace_r = np.asarray(trace_r)

    data = {"trace_t": trace_t, "trace_r": trace_r}
    filename = time.strftime("[%Y%m%d]%H%M", time.localtime())
    path = "/home/xj/Demo/THMP_Robot/thmp_solution/ds/{}.mat".format(filename)
    scio.savemat(path, data)
    print("saved file: {}".format(path))


if __name__ == "__main__":
    collection_human_trace()
