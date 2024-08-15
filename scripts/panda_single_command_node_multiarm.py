#!/usr/bin/env python3

import rospy
from robot_control.panda_robot_client_multi import panda_robot_client_multi
import numpy as np
# from robot_control.utils import *
import moveit_commander
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R
import std_msgs.msg


if __name__ == '__main__':
    rospy.init_node('single_move')

    group_name = 'left_arm'
    PRC_l = panda_robot_client_multi(group_name)

    group_name = 'right_arm'
    PRC_r = panda_robot_client_multi(group_name)

    #Giver
    T1 = [0.0, -np.pi / 4, 0.0, -2 * np.pi / 3, 0.0, np.pi / 3, np.pi / 4]
    S1 = PRC_r.moveToJoint(T1)
    print(S1)
    curr_pose = PRC_r.getPose('panda_1_hand_tcp')
    print('current pose is: ', curr_pose.pose)
    rospy.sleep(1.0)
    # target_pose = [0.8000360727310181, -0.09993565827608109, 0.40000033378601074, -0.7070422172546387, 3.444914000283461e-06, -0.7071710824966431, 0.0005641026655212045]
    # S2 = PRC_r.moveToPose(target_pose)
    # print(S2)
    # # curr_state = PRC_r.getJointStates()
    # # # print('current joint states is: ', curr_state.joints_state)
    stop_success = PRC_r.moveStop()
    width = 0.08#0.065/2
    S3 = PRC_r.moveGripper(width)

    rospy.sleep(1.0)

    # receiver
    T1 = [0.0, -np.pi / 4, 0.0, -2 * np.pi / 3, 0.0, np.pi / 3, np.pi / 4]
    S1 = PRC_l.moveToJoint(T1)
    print(S1)
    curr_pose = PRC_l.getPose('panda_2_hand_tcp')
    print('current pose is: ', curr_pose.pose)
    rospy.sleep(1.0)
    # receive_pose = [0.9231128219209699, 0.008585315056366394, 0.5047856297307995, -0.33361806866920696, -0.8488298486964116, 0.4055598492865107, -0.06089401338520825]
    # receive_pre_pose = [0.9398355094170613, 0.08149864478562609, 0.5711482552869085, -0.33361806866920696, -0.8488298486964116, 0.4055598492865107, -0.06089401338520825]
    # pre_grasp_pose = [8.933524782075167314e-01, -2.993982325438834108e-02, 3.083183632789497364e-01, 4.690950968489536188e-01, 1.568671852509679054e-01, -8.562277619431364339e-01, 1.490519908674723393e-01]
    # grasp_pose = [8.176983024775077435e-01, -7.078654266264072792e-02, 3.593868585396916004e-01, 4.690950968489536188e-01, 1.568671852509679054e-01, -8.562277619431364339e-01, 1.490519908674723393e-01]
    # S2 = PRC_l.moveToPose(grasp_pose)
    # print(S2)
    # # curr_state = PRC_l.getJointStates()
    # # print('current joint states is: ', curr_state.joints_state)
    stop_success = PRC_l.moveStop()
    width = 0.08#0.065/2
    S3 = PRC_l.moveGripper(width)