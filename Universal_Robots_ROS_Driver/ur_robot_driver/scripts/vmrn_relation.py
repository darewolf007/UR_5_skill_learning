import numpy as np
import os
import torch
import math
import re

from model.FasterRCNN_VMRN import fasterRCNN_VMRN
from model.utils.config import read_cfgs, cfg
from model.utils.blob import prepare_data_batch_from_cvimage
from model.utils.net_utils import rel_prob_to_mat, find_all_paths, create_mrt, objgrasp_inference, objdet_inference
from roi_data_layer.roidb import get_imdb, combined_roidb
from model.utils.data_viewer import dataViewer
from model.rpn.bbox_transform import bbox_xy_to_xywh

import cv2
from prob1.srv import VmrnRelation, VmrnRelationResponse
from prob1.msg import relationresult

import rospy
import pdb

class Relation_Demo(object):
    def __init__(self, args, model_path):
        cuda = True
        conv_num = str(int(np.log2(cfg.RCNN_COMMON.FEAT_STRIDE[0])))
        trained_model = torch.load(model_path)
        self.class_agnostic = trained_model['class_agnostic']
        # init VMRN
        cls_list = ['__background__',  # always index 0
                    'grenade', 'exploder', 'airplane', 'ashcan', 'bench', 'bookshelf', 'bottle', 'can',
                    'cap', 'helmet', 'mailbox', 'mug', 'pistol', 'pot', 'rocket', 'table', 'apple',
                    'cup', 'tool', 'foam', 'milk', 'spray', 'perfume', 'tape', 'rock', 'bowl']
        self.Only_Relation = fasterRCNN_VMRN(len(cls_list), class_agnostic=trained_model['class_agnostic'], feat_name=args.net,
                    feat_list=('conv2', 'conv3', 'conv4', 'conv5'), pretrained=True)
        self.Only_Relation.create_architecture(cfg.TRAIN.VMRN.OBJ_MODEL_PATH)
        self.Only_Relation.load_state_dict(trained_model['model'])
        if cuda:
            self.Only_Relation.cuda()
        # if args.cuda:
        #     self.Only_Relation.cuda()
            # self.cuda = True
        self.Only_Relation.eval()
        # init classes
        self.classes = cls_list
        self.class_to_ind = dict(zip(self.classes, range(len(cls_list))))
        self.ind_to_class = dict(zip(range(len(cls_list)), self.classes))

        # init data viewer
        self.data_viewer = dataViewer(self.classes)
        self.save_id = None

    def build_relationship_tree(self, id, obj_rois, obj_label, rels, clslist):
        rel_num = 0
        mani_tree = {}
        relfile = open('images/output'+ id +'_relation.txt','w')
        for i in range(obj_rois.size(0)):
            if i not in mani_tree.keys():
                mani_tree[i] = {}
                mani_tree[i]['child'] = []
                mani_tree[i]['parent'] = []
                mani_tree[i]['name'] = clslist[int(obj_label[i].item())]
                mani_tree[i]['bbox'] = obj_rois[i].cpu().numpy()
                mani_tree[i]['cls'] = int(obj_label[i].item())

            for ii in range(i+1, obj_rois.size(0)):
                if ii not in mani_tree.keys():
                    mani_tree[ii] = {}
                    mani_tree[ii]['child'] = []
                    mani_tree[ii]['parent'] = []
                    mani_tree[ii]['name'] = clslist[int(obj_label[ii].item())]
                    mani_tree[ii]['bbox'] = obj_rois[ii].cpu().numpy()
                    mani_tree[ii]['cls'] = int(obj_label[ii].item())
                rel = rels[rel_num]
                if rel == cfg.VMRN.FATHER:
                    # FATHER
                    print(clslist[int(obj_label[ii].item())] + '---->' + clslist[int(obj_label[i].item())])
                    relfile.write(clslist[int(obj_label[ii].item())] + '---->' + clslist[int(obj_label[i].item())] + '\n')
                    mani_tree[i]['child'].append(ii)
                    mani_tree[ii]['parent'].append(i)

                elif rel == cfg.VMRN.CHILD:
                    # CHILD
                    print(clslist[int(obj_label[i].item())] + '---->' + clslist[int(obj_label[ii].item())])
                    relfile.write(clslist[int(obj_label[i].item())] + '---->' + clslist[int(obj_label[ii].item())] + '\n')
                    mani_tree[i]['parent'].append(ii)
                    mani_tree[ii]['child'].append(i)

                else:
                    print(clslist[int(obj_label[ii].item())] + ' and ' +
                        clslist[int(obj_label[i].item())] + ' have no relationship.')
                    relfile.write(clslist[int(obj_label[ii].item())] + ' and ' +
                        clslist[int(obj_label[i].item())] + ' have no relationship.' + '\n')

                rel_num += 1
        relfile.close()
        return mani_tree

    def current_target(self, mani_tree, targets):
        childs = []
        for i in range(targets.size(0)):
            if len(mani_tree[targets[i].item()]['child']) == 0:
                return targets[i].item()
            else:
                childs += mani_tree[targets[i].item()]['child']
        # delete repeated elements
        childs = list(set(childs))
        return self.current_target(mani_tree, torch.LongTensor(childs).type_as(targets))

    def Relation_forward_process(self, image, save_res = False, id = "", target = ""):
        data_batch = prepare_data_batch_from_cvimage(image, is_cuda = True, frame="all_in_one")
        all_results = self.Only_Relation(data_batch)
        rel_result = all_results[3]
        obj_bboxes = rel_result[0]
        obj_classes = rel_result[1]
        rel_prob = rel_result[2]
        num_box = obj_bboxes.shape[0]
        rel_mat, rel_score = rel_prob_to_mat(rel_prob, num_box)
        if rel_prob.numel()>0:
            _, rel_prob = torch.max(rel_prob, dim = 1)
            rel_prob += 1
        obj_cls = []
        relation_tree = self.build_relationship_tree(id, obj_bboxes, obj_classes, rel_prob, self.classes)
        for cls in obj_classes.cpu().numpy():
            obj_cls.append(self.ind_to_class[cls])
        target_ind = self.class_to_ind[target]
        if torch.sum(obj_classes == target_ind).item() > 0:
            # the robot can see the target
            targets = torch.nonzero((obj_classes == target_ind))
            current_target_num = self.current_target(relation_tree, targets) #当前要抓目标的排序序号，例：[0,1,2,3]
            current_target = relation_tree[current_target_num]    
            current_target_ind = current_target['cls'] #当前要抓目标在cls_list中的序号，例：[1,4,5,7]
            current_target_name = self.ind_to_class[current_target_ind]
            # if current_cls != target_ind:
            #     for i in range(targets.size(0)):
            #         target_bbox = relation_tree[targets[i].item()]['bbox']              
            # obj = target_['bbox']
        else:
            # the robot cannot see the target
            # 1.随机选定当前目标（在没有子节点的情况下）
            for i, target_ in relation_tree.items():
                if len(target_['child']) == 0:
                    current_target_num = i
                    current_target_ind = target_['cls']
                    current_target_name = self.ind_to_class[current_target_ind]
                    break
            # 2.不抓了
            # current_target_num = -1
            # current_target_name = ''

        rois, cls_prob, bbox_pred, _, _, _, _, _, _, _, _, = all_results
        boxes = rois[:, :, 1:5]
        vis_boxes = objdet_inference(cls_prob[0].data,
                                        bbox_pred[0].data if bbox_pred is not None else bbox_pred,
                                        data_batch[1][0].data, boxes[0].data,
                                        class_agnostic=self.class_agnostic,
                                        for_vis=True, recover_imscale = True, with_cls_score = True) 

  

        if save_res:
            # obj_det_img = self.data_viewer.draw_graspdet_with_owner(image.copy(), vis_boxes,
            #                  vis_grasps.reshape(-1, vis_grasps.shape[-1]), g_inds)o
            # obj_det_img = self.data_viewer.draw_objdet(image.copy(), vis_boxes, o_inds=np.arange(vis_boxes.shape[0]))
            save_path = "images/output/"
            # cv2.imwrite(save_path + id + "_detection.jpg", obj_det_img)

            rel_det_img = self.data_viewer.draw_mrt(image.copy(), rel_mat, rel_score=rel_score)
            cv2.imwrite("images/output/" + id + "relation_det.png", rel_det_img)
        vis_boxes = vis_boxes[:,:-1]

        return vis_boxes, obj_cls, current_target_num, current_target_name

def rel_inference(index):
    # we need to read configs of VMRN that were used in training and also need to be used in this demo
    args = read_cfgs()
    args.frame = "faster_rcnn_vmrn"
    # load_name = '/home/xj/Documents/catkin_ws/src/prob1/scripts/vmrn/output/vmrdcompv1/res101/faster_rcnn_vmrn_1_1_20.pth'
    load_name = '/home/xj/Documents/catkin_ws/src/prob1/scripts/vmrn/output/vmrdcompv1/res101/faster_rcnn_vmrn_1_30_1629.pth'
    rel_demo = Relation_Demo(args, load_name)
    img_path = '/home/xj/Documents/catkin_ws/src/prob1/scripts/vmrn/images/example_' + str(index)+'.jpg'
    # img_path = '/home/xj/Documents/catkin_ws/src/prob1/scripts/vmrn/images/1254.jpg'
    target = 'grenade'
    cv_img = cv2.imread(img_path, cv2.IMREAD_COLOR)
    # VMRN forward process
    vis_boxes, obj_cls, current_target_num, current_target_name= rel_demo.Relation_forward_process(cv_img, save_res=True, id=os.path.split(img_path)[1][:-4], target=target)

    # pdb.set_trace()
    result = []
    for i in range(vis_boxes.shape[0]):
        result.append({'obj_boxes':vis_boxes[i], 
                        'obj_cls':obj_cls[i]
                        })
    result.append({'current_target':[current_target_num, current_target_name]})
    return result




class VMRNServiceRos_relation(object):
    def __init__(self):
        rospy.init_node('vmrn_rel_init')
        print('vmrn_rel_init service start')
        self.vmrndetect = rospy.Service('vmrn_relation', VmrnRelation, self.relation_detection)
        rospy.spin()

    def relation_detection(self, request):
        response = VmrnRelationResponse()
        if request.flag_detect:
            res = rel_inference(request.index)
            if len(res) > 0:
                res_tmp = relationresult()
                print(res[-1]['current_target'])
                res_tmp.current_target.id = res[-1]['current_target'][0]

                #res_tmp.obj_grasp.grasp = list(each_result['anchor_grasps'])
                response.results.append(res_tmp)
                print(response)
                return response
            else:
                return 0


if __name__ == '__main__':
    VMRNServiceRos_relation()