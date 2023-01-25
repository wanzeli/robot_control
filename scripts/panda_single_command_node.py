#!/usr/bin/env python3

import rospy
from robot_control.panda_robot_client import panda_robot_client
import numpy as np


if __name__ == '__main__':
    PRC = panda_robot_client()
    # T1 = [0.0, -np.pi/8, 0.0, -2*np.pi/4, 0.0, np.pi/1.1, np.pi/5]
    # T2 = [0.4, 0.1, 0.4, 0, 0, 0, 1]
    # width = 0.02
    # T1 = [0.0, -np.pi / 4, 0.0, -2 * np.pi / 3, 0.0, np.pi / 3, np.pi / 4]
    # T1 = [0.0, -np.pi / 4, 0.0, -2 * np.pi / 3, 0.0, np.pi / 3, np.pi / 4]
    T1 = [-0.0437932088971138, 0.20515790581703186, -0.06661075353622437, -1.8870114088058472, 1.3561750650405884, 1.7731914520263672, -0.194657102227211]
    S1 = PRC.moveToJoint(T1)
    # print(S1)
    # S2 = PRC.moveToPose(T2)
    # print(S2)
    # curr_pose = PRC.getPose()
    # print(curr_pose.pose)
    curr_state = PRC.getJointStates()
    print(curr_state.joints_state)
    # # S3 = PRC.moveGripper(width)
    # # print(S3)
    # gripper_state = PRC.getGripperStates()
    # print(gripper_state.gripper_state)
    # # stop_success = PRC.moveStop()
    # # print(stop_success)
    # object_path = '/home/wanze/hanging-point-detection/Sim_exp/src/panda_control/supporter_model/hook_wall_2/model_meshlabserver_normalized_wt.obj'
    # refer_frame = 'World'
    # object_id = 'supporter'
    # object_pose_list = [1,1,1,0,0,0,1]
    # size = (1,1,1)
    # object_list = PRC.add_mesh(object_path, refer_frame, object_id, object_pose_list, size)
    # print(object_list)

    # box_name = 'box'
    # refer_frame = 'world'
    # box_pose_list = [0.4,0,0.05,0,0,0,1]
    # box_size = (0.4, 0.2, 0.15)
    # object_list = PRC.add_box(box_name, refer_frame, box_pose_list, box_size)
    # print(object_list)
