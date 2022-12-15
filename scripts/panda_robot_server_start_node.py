import rospy
from robot_control.panda_robot_server import pandaRobotServer

if __name__ == "__main__":
    # initial ROS node: 
    rospy.init_node("panda_robot_server_start")
    # create server object
    Franka_server = pandaRobotServer()
    Franka_server.run_panda_service()
    curr_state = Franka_server.move_group.get_current_joint_values()
    print('current joint state is: ', curr_state)
    rospy.spin()