#!/usr/bin/env python3

import rospy
from robot_control.panda_robot_client import panda_robot_client
import numpy as np


if __name__ == '__main__':
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
    # S1 = PRC.moveToJoint(T1)
    # print(S1)

    # T2 = [0.37133273272372674, -0.043481634985421846, 0.24082379745621751, 0.9364627309890373, -0.33340443230036093, 0.06973846931779444, -0.08375908242237998]
    # T2 = [0.004585887771099806, 0.40176722407341003, 0.8177175521850586, -0.6473002433776855, -0.26359590888023376, -0.29538509249687195, 0.6513580083847046]
    
    # # T2 = [0.6393822431564331, 0.0031410674564540386, 0.5713240504264832, -0.6695536971092224, 0.2640250027179718, -0.6465916037559509, 0.2528001666069031]
    # T2 = [0.15, 0.3, 0.5592673633147, 0.486773551136288, -0.47951924717505795, 0.6268556890443894, -0.3743858258742136]
    # S2 = PRC.moveToPose(grasp_pose)
    # print(S2)

    # P1 = PRC.planToPose(grasp_pose)
    # print(P1)

    # T3 = [0.3, 0.3, 0.5592673633147, 0.486773551136288, -0.47951924717505795, 0.6268556890443894, -0.3743858258742136]
    # pre_place_pose = [0.6150994959022268, 0.07523005487653417, 0.809922364005978, 0.12496298395989684, -0.6909510755185653, 0.6985811909707228, 0.1376778250173312]
    # place_pose = [0.4865148663520813, -0.1013362854719162, 0.3914080560207367 + 0.2, -0.6926869750022888, 0.7211161851882935, 0.00023159988631960005, 0.013272421434521675]
    # S2 = PRC.moveToPose(place_pose)
    # print(S2)
    
    curr_pose = PRC.getPose()
    print(curr_pose.pose)
    # curr_state = PRC.getJointStates()
    # print(curr_state.joints_state)


    width = 0.06#0.065/2
    S3 = PRC.moveGripper(width)
    print(S3)
    # rospy.sleep(1)
    # # S1 = PRC.moveStop()
    # # print(S1)
    # width = 0.04 #0.065/2
    # S3 = PRC.moveGripper(width)
    # print(S3)
    # S1 = PRC.moveToJoint(T1)
    # print(S1)
    # S1 = PRC.moveToJoint(T1)
    # print(S1)
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

    # mesh_path = '/home/wanze/hanging-point-detection/Data_exp/RW_Experiment/Hanger/sword/hanger_mesh/final_mesh.obj'
    # object_id = 'hanger'
    # object_pose_list = [0, 0, 0, 0, 0, 0, 1]
    # size = (1,1,1)
    # attached_objects = PRC.attach_mesh(mesh_path, object_id, object_pose_list, size)
    # print(attached_objects)

    # SS = PRC.remove_attach_mesh('hanger')
    # print(SS)
    mesh_path = '/home/wanze/hanging-point-detection/Data_exp/RW_Experiment/Hanger/sword/hanger_mesh/final_mesh.obj'
    refer_frame = 'world'
    obj_name = 'hanger'
    object_pose_list = [0, 0, 0, 0, 0, 0, 1]
    object_size = (1,1,1)
    mesh_list = PRC.add_mesh(mesh_path, refer_frame, obj_name, object_pose_list, object_size)
    print(mesh_list)


    # box_name = 'box-up'
    # refer_frame = 'world'
    # box_pose_list = [0.455,-0.01,0.2185,0,0,0,1]
    # box_size = (0.30, 0.29, 0.097)
    # object_list = PRC.add_box(box_name, refer_frame, box_pose_list, box_size)
    # print(object_list)

    # box_name = 'box-down'
    # refer_frame = 'world'
    # box_pose_list = [0.455,-0.01,0.085,0,0,0,1]
    # box_size = (0.40, 0.29, 0.17)
    # object_list = PRC.add_box(box_name, refer_frame, box_pose_list, box_size)
    # print(object_list)

    # box_name = 'wall_v'
    # refer_frame = 'world'
    # box_pose_list = [0.82,-0.15,0.40,0,0,0,1]
    # box_size = (0.04, 0.04, 0.80)
    # object_list = PRC.add_box(box_name, refer_frame, box_pose_list, box_size)
    # print(object_list)

    # box_name = 'wall_h'
    # refer_frame = 'world'
    # box_pose_list = [0.82,-0.015,0.63,0,0,0,1]
    # box_size = (0.04, 0.23, 0.10)
    # object_list = PRC.add_box(box_name, refer_frame, box_pose_list, box_size)
    # print(object_list)

    # box_name = 'wall'
    # refer_frame = 'world'
    # box_pose_list = [0.82,0.30,0.40,0,0,0,1]
    # box_size = (0.04, 0.26, 0.80)
    # object_list = PRC.add_box(box_name, refer_frame, box_pose_list, box_size)
    # print(object_list)

    # box_name = '4040_left'
    # refer_frame = 'world'
    # box_pose_list = [0.87,-0.29,0.44,0,0,0,1]
    # box_size = (0.04, 0.04, 0.88)
    # object_list = PRC.add_box(box_name, refer_frame, box_pose_list, box_size)
    # print(object_list)

    # box_name = '4040_horizon'
    # refer_frame = 'world'
    # box_pose_list = [0.87,-0.02,0.72,0,0,0,1]
    # box_size = (0.04, 0.50, 0.04)
    # object_list = PRC.add_box(box_name, refer_frame, box_pose_list, box_size)
    # print(object_list)

    # box_name = '4040_right'
    # refer_frame = 'world'
    # box_pose_list = [0.87,0.25,0.44,0,0,0,1]
    # box_size = (0.04, 0.04, 0.88)
    # object_list = PRC.add_box(box_name, refer_frame, box_pose_list, box_size)
    # print(object_list)

    # box_name = '4040'
    # refer_frame = 'world'
    # box_pose_list = [0.37,0.3,0.25,0,0,0,1]
    # box_size = (0.04, 0.04, 0.50)
    # object_list = PRC.add_box(box_name, refer_frame, box_pose_list, box_size)
    # print(object_list)