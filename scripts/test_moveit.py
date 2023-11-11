#!/usr/bin/env python3
import rospy
import moveit_commander

if __name__ == '__main__':
    rospy.init_node('test_moveit')
    move_group = moveit_commander.MoveGroupCommander('panda_arm')