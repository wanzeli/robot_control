import rospy
from robot_control.panda_robot_server import pandaRobotServer
from robot_control.panda_robot_server_multi import pandaRobotServerMulti

if __name__ == "__main__":
    # initial ROS node: 
    rospy.init_node("panda_robot_server_start")
    group_name = 'panda_arm' 
    group_hand_name = 'panda_hand' 
    Franka_server = pandaRobotServer(group_name, group_hand_name)
    Franka_server.run_panda_service()
    rospy.sleep(1.0)
    # curr_pose = Franka_server_right.move_group.get_current_pose()
    # print('current pose for ', group_name, ' is : ', curr_pose)
    # curr_state = Franka_server_right.move_group.get_current_joint_values()
    # print('current joint state for ', group_name, ' is: ', curr_state)

    # # create server object for right arm
    # group_name = 'right_arm' 
    # group_hand_name = 'right_hand' 
    # panda_id = 'panda_1'
    # Franka_server_right = pandaRobotServerMulti(group_name, group_hand_name, panda_id)
    # Franka_server_right.run_panda_service()
    # rospy.sleep(1.0)
    # curr_pose = Franka_server_right.move_group.get_current_pose()
    # print('current pose for ', group_name, ' is : ', curr_pose)
    # curr_state = Franka_server_right.move_group.get_current_joint_values()
    # print('current joint state for ', group_name, ' is: ', curr_state)

    # # create server object for left arm
    # group_name = 'left_arm' #'right_arm' #
    # group_hand_name = 'left_hand' #'right_hand' #
    # panda_id = 'panda_2' #'panda_1' #
    # Franka_server_left = pandaRobotServerMulti(group_name, group_hand_name, panda_id)
    # Franka_server_left.run_panda_service()
    # rospy.sleep(1.0)
    # curr_pose = Franka_server_left.move_group.get_current_pose()
    # print('current pose for ', group_name, ' is : ', curr_pose)
    # curr_state = Franka_server_left.move_group.get_current_joint_values()
    # print('current joint state for ', group_name, ' is: ', curr_state)

    rospy.spin()