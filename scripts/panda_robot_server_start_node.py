import rospy
from robot_control.panda_robot_server import pandaRobotServer

if __name__ == "__main__":
    # initial ROS node: 
    rospy.init_node("panda_robot_server_start")
    # # create server object
    # Franka_server = pandaRobotServer()
    # Franka_server.run_panda_service()
    # curr_state = Franka_server.move_group.get_current_joint_values()
    # print('current joint state is: ', curr_state)

    right_arm_name = 'right_arm'   
    right_hand_name = 'right_hand' 
    left_arm_name = 'left_arm'
    left_hand_name = 'left_hand'
    Franka_server_right = pandaRobotServer(right_arm_name, right_hand_name)
    Franka_server_left = pandaRobotServer(left_arm_name, left_hand_name)

    Franka_server_right.run_panda_service()
    curr_state_right = Franka_server_right.move_group.get_current_joint_values()
    print('current joint state for right arm is: ', curr_state_right)

    # Franka_server_left.run_panda_service()
    # curr_state_right = Franka_server_left.move_group.get_current_joint_values()
    # print('current joint state for left arm is: ', curr_state_right)

    rospy.spin()