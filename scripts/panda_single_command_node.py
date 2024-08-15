#!/usr/bin/env python3

import rospy
from robot_control.panda_robot_client import panda_robot_client
import numpy as np
# from robot_control.utils import *
import moveit_commander
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R
import std_msgs.msg

class dumb_place(): 

    def __init__(self, group_name='panda_arm', group_hand_name='panda_hand', force_topic='/franka_state_controller/F_ext'):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.move_group_hand = moveit_commander.MoveGroupCommander(group_hand_name)
        self.eef_link = self.move_group.get_end_effector_link()

        # set up the subscriber to get force: 
        self.forceSub = rospy.Subscriber(force_topic, geometry_msgs.msg.WrenchStamped, self.getForceCb)
        self.force = []
        self.force_control_flag = 0
        self.force_thr = -5
        self.stop = False
    
    def getForceCb(self, msg): 
        '''
        This function save the force 
        '''
        if msg is None: 
            rospy.logwarn('forceCb: msg is None!')
        else: 
            Force_msg = msg.wrench.force
            self.force = [Force_msg.x, Force_msg.y, Force_msg.z]
            pose = self.move_group.get_current_pose()
            quat = [pose.pose.orientation.x, 
                    pose.pose.orientation.y, 
                    pose.pose.orientation.z, 
                    pose.pose.orientation.w]
            r = R.from_quat(quat)
            Rot = r.as_matrix()
            force_w = Rot @ np.asarray(self.force) # transform the force from ee link to world
            if force_w[2] < self.force_thr and self.force_control_flag == 1: 
                print('Hanger contacts with supporter')
                print(force_w)
                self.move_group.stop()
                self.stop = True 

    def go_to_pose(self, target=None):

        pose_goal = Pose()
        pose_goal.position.x = target[0]
        pose_goal.position.y = target[1]
        pose_goal.position.z = target[2]
        pose_goal.orientation.x = target[3]
        pose_goal.orientation.y = target[4]
        pose_goal.orientation.z = target[5]
        pose_goal.orientation.w = target[6]

        self.move_group.set_pose_target(pose_goal)
        move_success = self.move_group.go(wait=False)

        if move_success:
            success = 1
            rospy.loginfo("Success")
        else:
            success = 0
            rospy.loginfo("Failure")
        
        return success

if __name__ == '__main__':
    rospy.init_node('single_move')
    PRC = panda_robot_client()
    # T1 = [0.0, -np.pi/8, 0.0, -2*np.pi/4, 0.0, np.pi/1.1, np.pi/5]
    # T2 = [0.4, 0.1, 0.4, 0, 0, 0, 1]
    # width = 0.02
    T1 = [0.0, -np.pi / 4, 0.0, -2 * np.pi / 3, 0.0, np.pi / 3, np.pi / 4]
    grasp_pose = [0.4735301434993744, -0.022408511489629745, 0.4115336835384369, -0.9117316603660583, -0.4099779427051544, 0.02216835506260395, 0.01311434991657734]
    pre_grasp_pose = [0.49345189332962036, -0.02387768216431141, 0.4, 0.917822003364563, -0.395513117313385, -0.03175073117017746, 0.012807437218725681]
    pre_place_pose = [0.6167065188507247, 0.037537305824227596, 0.3287496909581556, 0.2885027003250105, 0.6272323513605325, 0.41285021831548496, 0.5940542622932444]
    place_pose = [0.7667065188507247, 0.037537305824227596, 0.3287496909581556, 0.2885027003250105, 0.6272323513605325, 0.41285021831548496, 0.5940542622932444]
    # T1 = [-0.23545381139294091, 0.6116140798897288, -0.10910477402588925, -1.1215665589752069, 0.7069849459673045, 3.289424702186856, 1.006700248219855]
    # T1 = [0.0415367854493156, -0.228943687900871, -0.021118349279214126, -1.8345489732187341, -0.09314659575659194, 2.6222007863948935, 0.958800150644423]
    # T1 = [-0.0437932088971138, 0.20515790581703186, -0.06661075353622437, -1.8870114088058472, 1.3561750650405884, 1.7731914520263672, -0.194657102227211]
    # T1 = [-0.674559, 0.350548, -1.42584, -2.1507, 1.90127, 1.47858, -0.177635]
    # T1 = [-0.05345608078674497, 0.389683296716559, -0.5925520371346319, -0.8730247469830527, 0.0027175449127635675, 2.494913541175824, 1.924357712436688]
    # T1 = [0.3979649498704121, 0.5254917693683857, -0.13393714383022945, -0.8246807102577624, -0.33605845828569203, 3.226814633597608, 0.34642188363853016]
    # T1 = [-0.05345608078674497, 0.389683296716559, -0.5925520371346319, -0.8730247469830527, 0.0027175449127635675, 2.494913541175824, 1.924357712436688]
    # T1 = [-0.01081784258611293, 0.3707728067072197, 0.04866113101581512, -0.7985380527322543, -0.0010330420840473221, 3.2085378766966883, 0.7021069491585636]
    # T1 = [0.6062635325469495, 0.5628082866126526, -0.42540124154620534, -0.7892652468504394, -0.35572012383542356, 3.198372478749012, 0.5271382364222094]
    # T1 = [-0.05345608078674497, 0.389683296716559, -0.5925520371346319, -0.8730247469830527, 0.0027175449127635675, 2.494913541175824, 1.924357712436688]
    # scan_pose = [0.016482016203150065, 0.28071231262161456, 0.012837786511849823, -0.7553353200282434, -0.024522649614004417, 3.072981311953512, 0.7408528760270671]
    # T1 = [0.028032411973095212, 0.0012298313257931973, -0.004756620343743241, -1.6155257910153364, -0.09252743389476417, 2.629796528390364, 0.9642478784252707]
    # T1 = [-1.6451903396268086, -0.95278068958986, 1.6874757774530067, -1.48463790317552, 0.2682014430626306, 2.5719663888302287, 0.6725364658121776]
    # T1 = [-0.13912776112556458, 0.38246676325798035, -0.3782239258289337, -1.0559041500091553, 0.6242861151695251, 2.890017032623291, 1.2693341970443726]
    # T1 = [0.1, -np.pi / 4, 0.0, -2 * np.pi / 3, 0.0, np.pi / 3, np.pi / 4]
    # T1 = [0.007464191876351833, -0.10961239039897919, -0.21164244413375854, -2.3060545921325684, -0.0010407171212136745, 2.214181900024414, 1.409178614616394]
    # T1 = [-0.2164285033941269, 0.3437270224094391, -0.45246851444244385, -0.9604554772377014, 0.746423065662384, 2.889063596725464, 1.1668925285339355]
    # T1 = [0.1, -np.pi / 4, 0.0, -2 * np.pi / 3, 0.0, np.pi / 3, np.pi / 4]
    # # T1 = [0.36398524045944214, 0.39771226048469543, -0.5281479954719543, -0.6297473311424255, 0.25043225288391113, 3.1724419593811035, 0.6444401741027832]
    # T1 = [-0.5978644490242004, 0.165483757853508, -0.6527774333953857, -1.2522087097167969, 1.3006049394607544, 2.4343881607055664, 0.7244192957878113]
    # T1 = [-0.009746570315308109, 0.6015037727721495, 0.03503112650072337, -1.6787263831435038, 0.1683203640369969, 2.633258973865727, 0.6464663715574538 - np.pi]
    # S1 = PRC.moveToJoint(T1)
    # print(S1)

    # width = 0.03#0.065/2
    # S3 = PRC.moveGripper(width)
    # stop_success = PRC.moveStop()

    curr_pose = PRC.getPose()
    print(curr_pose.pose)
    # cali_pose = [0.5, 0, 0.18, curr_pose.pose[3], curr_pose.pose[4], curr_pose.pose[5], curr_pose.pose[6]] #0.206
    # cap_pose = [curr_pose.pose[0], curr_pose.pose[1], curr_pose.pose[2]-0.1, curr_pose.pose[3], curr_pose.pose[4], curr_pose.pose[5], curr_pose.pose[6]]
    # S2 = PRC.moveToPose(cap_pose)
    # print(S2)


