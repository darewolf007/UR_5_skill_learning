#!/usr/bin/env python2
#coding=utf-8
import rospy
from prob1.srv import VmrnDetection

def get_vmrn_results():
    rospy.wait_for_service('vmrn_detection')
    print('vmrn detection.......')
    try:
        obj_client = rospy.ServiceProxy('vmrn_detection',VmrnDetection)
        response = obj_client.call(True)
        return response.results
    except rospy.ServiceException as e:
        rospy.loginfo("service call failed:%s"%e)

if __name__ == "__main__":
    rospy.init_node('aaaa')
    re = get_vmrn_results()
    print(re)