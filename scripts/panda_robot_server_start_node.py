import rospy
from robot_control.panda_robot_server import pandaRobotServer

if __name__ == "__main__":
    # initial ROS node: 
    rospy.init_node("panda_robot_server_start")
    # create server object
    group_name = 'left_arm'
    group_hand_name = 'left_hand'
    panda_id = 'panda_2'
    Franka_server = pandaRobotServer(group_name, group_hand_name, panda_id)
    Franka_server.run_panda_service()
    rospy.sleep(1.0)
    curr_pose = Franka_server.move_group.get_current_pose()
    print('current pose is: ', curr_pose)

    curr_state = Franka_server.move_group.get_current_joint_values()
    print('current joint state is: ', curr_state)
    rospy.spin()