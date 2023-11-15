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
    PRC = panda_robot_client_multi(group_name)
    T1 = [0.0, -np.pi / 4, 0.0, -2 * np.pi / 3, 0.0, np.pi / 3, np.pi / 4]
    S1 = PRC.moveToJoint(T1)
    print(S1)
    curr_pose = PRC.getPose('')
    print('current pose is: ', curr_pose.pose)
    rospy.sleep(1.0)
    target_pose = [curr_pose.pose[0], curr_pose.pose[1], curr_pose.pose[2] + 0.1, curr_pose.pose[3], curr_pose.pose[4], curr_pose.pose[5], curr_pose.pose[6]]
    S2 = PRC.moveToPose(target_pose)
    print(S2)
    curr_state = PRC.getJointStates()
    print('current joint states is: ', curr_state.joints_state)
    stop_success = PRC.moveStop()
    width = 0.06#0.065/2
    S3 = PRC.moveGripper(width)

    rospy.sleep(1.0)

    group_name = 'right_arm'
    PRC = panda_robot_client_multi(group_name)
    T1 = [0.0, -np.pi / 4, 0.0, -2 * np.pi / 3, 0.0, np.pi / 3, np.pi / 4]
    S1 = PRC.moveToJoint(T1)
    print(S1)
    curr_pose = PRC.getPose('')
    print('current pose is: ', curr_pose.pose)
    rospy.sleep(1.0)
    target_pose = [curr_pose.pose[0], curr_pose.pose[1], curr_pose.pose[2] + 0.1, curr_pose.pose[3], curr_pose.pose[4], curr_pose.pose[5], curr_pose.pose[6]]
    S2 = PRC.moveToPose(target_pose)
    print(S2)
    curr_state = PRC.getJointStates()
    print('current joint states is: ', curr_state.joints_state)
    stop_success = PRC.moveStop()
    width = 0.06#0.065/2
    S3 = PRC.moveGripper(width)