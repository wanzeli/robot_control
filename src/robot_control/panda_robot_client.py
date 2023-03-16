#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from robot_control.srv import move2joint, moveGripper, move2pose, stop, getJoints, getGripper, getPose, getForce, addMesh, addBox, attachMesh, removeMesh, removeObject
import numpy as np

'''
This program create clients to call the panda_robot server to execute the robot action
'''
class panda_robot_client():
    def __init__(self):
        pass

    def moveToJoint(self, target): 
        '''
        move the panda arm to a specific joint state
        target: a 7D list, specify the 7 joint states of robot 
        '''
        rospy.wait_for_service("panda_move_to_joint_state_server")
        target_state = JointState()
        target_state.position = target
        move_to_joint_client = rospy.ServiceProxy('panda_move_to_joint_state_server', move2joint)
        success = move_to_joint_client(target_state)

        return success

    def moveToPose(self, target): 
        '''
        move the panda arm to a specific pose of the end effector
        target: a 7D list, specify the target pose of the endeffector: [x,y,z,ox,oy,oz,w]
        '''        
        rospy.wait_for_service("panda_move_to_pose_server")
        pose_goal = Pose()
        pose_goal.position.x = target[0]
        pose_goal.position.y = target[1]
        pose_goal.position.z = target[2]
        pose_goal.orientation.x = target[3]
        pose_goal.orientation.y = target[4]
        pose_goal.orientation.z = target[5]
        pose_goal.orientation.w = target[6]
        move_to_pose_client = rospy.ServiceProxy("panda_move_to_pose_server", move2pose)
        success = move_to_pose_client(pose_goal)

        return success

    def planToPose(self, target): 
        '''
        plan the panda arm to a specific pose of the end effector
        target: a 7D list, specify the target pose of the endeffector: [x,y,z,ox,oy,oz,w]
        '''        
        rospy.wait_for_service("panda_plan_to_pose_server")
        pose_goal = Pose()
        pose_goal.position.x = target[0]
        pose_goal.position.y = target[1]
        pose_goal.position.z = target[2]
        pose_goal.orientation.x = target[3]
        pose_goal.orientation.y = target[4]
        pose_goal.orientation.z = target[5]
        pose_goal.orientation.w = target[6]
        plan_to_pose_client = rospy.ServiceProxy("panda_plan_to_pose_server", move2pose)
        success = plan_to_pose_client(pose_goal)

        return success

    def moveGripper(self, width): 
        '''
        move the panda hand gripper to specific position
        width: a float number, specific target width of the gripper
        '''        
        rospy.wait_for_service("panda_move_gripper_server")
        move_gripper_client = rospy.ServiceProxy('panda_move_gripper_server', moveGripper)
        success = move_gripper_client(width)

        return success

    def moveStop(self): 
        rospy.wait_for_service("panda_stop_server")
        move_stop_client = rospy.ServiceProxy('panda_stop_server', stop)
        stop_success = move_stop_client()

        return stop_success

    def getJointStates(self): 
        rospy.wait_for_service("panda_get_states")
        get_state_client = rospy.ServiceProxy('panda_get_states', getJoints)
        joints_state = get_state_client()    

        return joints_state

    def getGripperStates(self): 
        rospy.wait_for_service("panda_get_gripper")
        get_gripper_client = rospy.ServiceProxy('panda_get_gripper', getGripper)
        gripper_state = get_gripper_client()    

        return gripper_state
    
    def getPose(self, link_name = ''): 
        rospy.wait_for_service('panda_get_pose')
        get_pose_client = rospy.ServiceProxy('panda_get_pose', getPose)
        curr_pose = get_pose_client(link_name)

        return curr_pose
    
    def getForce(self): 
        rospy.wait_for_service('panda_get_force')
        get_force_client = rospy.ServiceProxy('panda_get_force', getForce)
        curr_force = get_force_client()

        return curr_force

    def add_mesh(self, object_path, refer_frame, object_id, object_pose_list, size = (1,1,1)): 
        rospy.wait_for_service('add_mesh')
        add_mesh_client = rospy.ServiceProxy('add_mesh', addMesh)
        objects_in_scene = add_mesh_client(object_path, refer_frame, object_id, object_pose_list, size)

        return objects_in_scene

    def add_box(self, box_name, refer_frame, box_pose_list, box_size): 
        rospy.wait_for_service('add_box')
        add_box_client = rospy.ServiceProxy('add_box', addBox)
        objects_in_scene = add_box_client(box_name, refer_frame, box_pose_list, box_size)

        return objects_in_scene

    def attach_mesh(self, mesh_path, object_id, object_pose_list, size): 
        rospy.wait_for_service('attach_mesh')
        attach_mesh_client = rospy.ServiceProxy('attach_mesh', attachMesh)
        attached_objects = attach_mesh_client(mesh_path, object_id, object_pose_list, size)

        return attached_objects

    def remove_attach_mesh(self, mesh_name): 
        rospy.wait_for_service('detach_mesh')
        detach_mesh_client = rospy.ServiceProxy('detach_mesh', removeMesh)
        success = detach_mesh_client(mesh_name)

        return success

    def remove_object(self, object_id): 
        rospy.wait_for_service('remove_object')
        remove_object_client = rospy.ServiceProxy('remove_object', removeObject)
        success = remove_object_client(object_id)

        return success

if __name__ == '__main__':
    PRC = panda_robot_client()
    T1 = [0.0, -np.pi/8, 0.0, -2*np.pi/4, 0.0, np.pi/1.1, np.pi/5]
    # T2 = [0.4, 0.1, 0.4, 0, 0, 0, 1]
    # width = 0.02
    S1 = PRC.moveToJoint(T1)
    print(S1)
    # S2 = PRC.moveToPose(T2)
    # print(S2)
    curr_pose = PRC.getPose()
    print(curr_pose.pose)
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
    # refer_frame = 'World'
    # box_pose_list = [1,1,2,0,0,0,1]
    # box_size = (0.5, 0.5, 0.5)
    # object_list = PRC.add_box(box_name, refer_frame, box_pose_list, box_size)
    # print(object_list)
