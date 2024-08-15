# -*- coding: utf-8 -*-

def pose_to_list_my(pose):  # [xyz, xyzw]
    return [pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
