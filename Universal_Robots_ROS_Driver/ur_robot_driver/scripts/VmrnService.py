#disable _ 20220422 !/usr/bin/env python2
#coding=utf-8
import imp
import re
import numpy as np
import os
import torch
import math

# from model.AllinOne import All_in_One
from model.AllinOne import All_in_One
from model.utils.config import read_cfgs, cfg
from model.utils.blob import prepare_data_batch_from_cvimage
from model.utils.net_utils import rel_prob_to_mat, find_all_paths, create_mrt, objgrasp_inference
from roi_data_layer.roidb import get_imdb, combined_roidb
from model.utils.data_viewer import dataViewer
from model.rpn.bbox_transform import bbox_xy_to_xywh

from prob1.srv import VmrnDetection, VmrnDetectionResponse
from prob1.msg import vmrnresult

import cv2

import rospy
import pdb


class AllinOneDemo(object):
    def __init__(self, args, model_path):
        cuda = True
        conv_num = str(int(np.log2(cfg.RCNN_COMMON.FEAT_STRIDE[0])))
        trained_model = torch.load(model_path)
        self.class_agnostic = trained_model['class_agnostic']
        # init VMRN
        # _, _, _, _, cls_list = combined_roidb(args.imdbval_name, training=False)
        cls_list = ['__background__',  # always index 0
                    'grenade', 'exploder', 'airplane', 'ashcan', 'bench', 'bookshelf', 'bottle', 'can',
                    'cap', 'helmet', 'mailbox', 'mug', 'pistol', 'pot', 'rocket', 'table', 'apple',
                    'cup', 'tool', 'foam', 'milk', 'spray', 'perfume', 'tape', 'rock', 'bowl']
        self.AllinOne = All_in_One(len(cls_list), class_agnostic=trained_model['class_agnostic'], feat_name=args.net,
                    feat_list=('conv2', 'conv3', 'conv4', 'conv5'), pretrained=True)
        print("!!!!!!!!!!!!!!!!-----1")
        self.AllinOne.create_architecture(cfg.MGN.OBJ_MODEL_PATH)
        print("!!!!!!!!!!!!!!!!-----2")
        self.AllinOne.load_state_dict(trained_model['model'])
        print("!!!!!!!!!!!!!!!!-----3")
        # print("aaaaaaaaa",self.AllinOne.load_state_dict(trained_model['model']))
        if cuda:
            self.AllinOne.cuda()
        #     self.cuda = True
        self.AllinOne.eval()
        # init classes
        self.classes = cls_list
        self.class_to_ind = dict(zip(self.classes, range(len(cls_list))))
        self.ind_to_class = dict(zip(range(len(cls_list)), self.classes))

        # init data viewer
        self.data_viewer = dataViewer(self.classes)
        self.save_id = None

    def AllinOne_forward_process(self, image, save_res = False, id = ""):
        data_batch = prepare_data_batch_from_cvimage(image, is_cuda = True, frame="all_in_one")
        all_results = self.AllinOne(data_batch)
        rel_result = all_results[3]
        obj_bboxes = rel_result[0].cpu().numpy()
        obj_classes = rel_result[1].cpu().numpy()
        num_box = obj_bboxes.shape[0]
        rel_prob = rel_result[2]
        rel_mat, rel_score = rel_prob_to_mat(rel_prob, num_box)
        obj_cls_name = []
        for cls in obj_classes:
            obj_cls_name.append(self.ind_to_class[cls])
        mrt = create_mrt(rel_mat, rel_score=rel_score)

        rois, cls_prob, bbox_pred, _, _, _, _, _, _, _, _, \
        grasp_loc, grasp_prob, _, _, _, grasp_all_anchors = all_results
        boxes = rois[:, :, 1:5]
        vis_boxes, vis_grasps = objgrasp_inference(cls_prob[0].data,
                                                   bbox_pred[0].data if bbox_pred is not None else bbox_pred,
                                                   grasp_prob.data, grasp_loc.data, data_batch[1][0].data,
                                                   boxes[0].data,
                                                   class_agnostic=self.class_agnostic,
                                                   g_box_prior=grasp_all_anchors.data, for_vis=True, topN_g=3)
        obj_cls = []
        for i in range(vis_boxes.shape[0]):
            obj_cls.append(self.data_viewer.ind_to_class[int(vis_boxes[i, -1])])

        vis_grasps = vis_grasps[:,0,:]   
        coodinate_grasps = vis_grasps
        anchor_grasps = []

        for i, coodinates in enumerate(vis_grasps):
            x0 = coodinates[0]
            y0 = coodinates[1]
            for i in range(4):
                if coodinates[i*2] < x0:
                    x0 = coodinates[i*2]
                    y0 = coodinates[i*2 + 1]
            distance = {}
            for i in range(4):
            #    distance.append({i:math.sqrt((x0-coodinates[i])**2+(y0-coodinates[i+1])**2)})   
                distance[i*2] = math.sqrt((x0-coodinates[i*2])**2+(y0-coodinates[i*2+1])**2)        
            length = sorted(distance.values(), reverse=True)[1]
            m = list(distance.keys())[list(distance.values()).index(length)]
            x1 = coodinates[m]
            y1 = coodinates[m+1]
            angle = math.atan((y0-y1)/(x1-x0))
            angle = angle * 180 / np.pi
            diagonal = sorted(distance.values(), reverse=True)[0]
            n = list(distance.keys())[list(distance.values()).index(diagonal)]           
            x2 = coodinates[n]
            y2 = coodinates[n+1]
            x_center = (x0 + x2) / 2
            y_center = (y0 + y2) / 2
            anchor_grasps.append([x_center, y_center, angle])


        if save_res:
            if vis_boxes.shape[0] > 0:
                g_inds = np.tile(np.expand_dims(np.arange(vis_boxes.shape[0]), 1), (1, vis_grasps.shape[1]))
            else:
                g_inds = np.array([], dtype=np.int32)
            g_inds = g_inds.reshape(-1)
            obj_det_img = self.data_viewer.draw_graspdet_with_owner(image.copy(), vis_boxes,
                             vis_grasps.reshape(-1, vis_grasps.shape[-1]), g_inds)
            # obj_det_img = self.data_viewer.draw_objdet(image.copy(), vis_boxes, o_inds=np.arange(vis_boxes.shape[0]))
            save_path = "images/output/"
            cv2.imwrite(save_path + id + "_detection.jpg", obj_det_img)
            #cv2.imwrite(save_path + "101_detection.jpg", obj_det_img)

            # rel_det_img = self.data_viewer.draw_mrt(image.copy(), rel_mat, rel_score=rel_score)
            # cv2.imwrite("images/output/" + id + "relation_det.png", rel_det_img)
        vis_boxes = vis_boxes[:,:-1]

        return vis_boxes, obj_cls, mrt, coodinate_grasps, anchor_grasps

def inference(index):
 #def inference():
    # we need to read configs of VMRN that were used in training and also need to be used in this demo
    args = read_cfgs()
    # model_dir = os.path.join(args.save_dir + "/" + args.dataset + "/" + args.net)
    model_dir = '/home/xj/Documents/catkin_ws/src/prob1/scripts/vmrn/output/vmrdcompv1/res101'
    load_name = os.path.join(model_dir, 'all_in_one_1_24_1701.pth')#all_in_one_1_21_197
    all_in_one_demo = AllinOneDemo(args, load_name)
    demo_mode = "all"

    if not os.path.exists("images/output"):
        os.makedirs("images/output")

    if demo_mode == "interactive":
        # interactive demo
        while True:
            image_id = input('Image ID: ').lower()
            if image_id == 'break':
                break
            # read cv image
            test_img_path = os.path.join('images', image_id + ".jpg")
            if not os.path.exists(test_img_path):
                print("The image ID you have input does not exist. Please check the path.")
                continue
            cv_img = cv2.imread(test_img_path, cv2.IMREAD_COLOR)
            # VMRN forward process
            vis_boxes, obj_cls, mrt, coodinate_grasps, anchor_grasps = all_in_one_demo.AllinOne_forward_process(cv_img, save_res=True, id = image_id)
        
    else:
        img_path = '/home/xj/Documents/catkin_ws/src/prob1/scripts/vmrn/images/example_' + str(index)+'.jpg'
        # img_path = '/home/xj/Documents/catkin_ws/src/prob1/scripts/vmrn/images/1254.jpg'
        cv_img = cv2.imread(img_path, cv2.IMREAD_COLOR)
        #pdb.set_trace()
        # VMRN forward process
        vis_boxes, obj_cls, mrt, coodinate_grasps, anchor_grasps = all_in_one_demo.AllinOne_forward_process(cv_img, save_res=True, id=os.path.split(img_path)[1][:-4])
        result = []
        for i in range(vis_boxes.shape[0]):
            result.append({'obj_boxes':vis_boxes[i], 
                            'obj_cls':obj_cls[i],
                            'coodinate_grasps': coodinate_grasps[i],
                            'anchor_grasps': anchor_grasps[i]
                            })
        return result


class VMRNServiceRos(object):
    def __init__(self):
        rospy.init_node('vmrn_det_init')
        print('vmrn_det_init service start')
        self.vmrndetect = rospy.Service('vmrn_detection', VmrnDetection, self.detection)
        rospy.spin()

    def detection(self, request):
        response = VmrnDetectionResponse()
        if request.flag_detect:
            res = inference(request.index)
            if len(res) > 0:
                for each_result in res:
                    res_tmp = vmrnresult()
                    res_tmp.obj_cls = each_result['obj_cls']
                    
                    res_tmp.obj_bbox.xyxy = list(each_result['obj_boxes'])
                    res_tmp.obj_grasp.grasp = list(each_result['anchor_grasps'])
                    response.results.append(res_tmp)
                    print(response)
                return response
            else:
                return 0
        

        

if __name__ == '__main__':
    # load_name = os.path.join( '/home/xj/Documents/catkin_ws/src/prob1/scripts/vmrn/output/vmrdcompv1/res101', 'all_in_one_1_21_1768.pth')
   
    #res  = inference()
    #print(res)
    VMRNServiceRos()