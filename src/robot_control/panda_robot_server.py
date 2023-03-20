#!/usr/bin/env python3

import rospy
import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg
from robot_control.srv import move2joint, move2pose, moveGripper, stop, getJoints, getGripper, getPose, addMesh, addBox, attachMesh, removeMesh, getForce, removeObject
import time
import actionlib
import franka_gripper.msg
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from scipy.spatial.transform import Rotation as R
import numpy as np

class pandaRobotServer():

    def __init__(self, group_name='panda_arm', group_hand_name='panda_hand', force_topic='/franka_state_controller/F_ext'):

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.move_group_hand = moveit_commander.MoveGroupCommander(group_hand_name)
        self.eef_link = self.move_group.get_end_effector_link()
        print('the end effector link is: ', self.eef_link)

        self.allowReplanning()
        r_ground = self.addGround()

        # set up the subscriber to get force: 
        
        self.forceSub = rospy.Subscriber(force_topic, geometry_msgs.msg.WrenchStamped, self.getForceCb)
        self.force = []
        

    def go_to_joint_state(self, joint_goal=None):

        move_group = self.move_group
        move_success = move_group.go(joint_goal, wait=True)
        move_group.stop()

        if move_success:
            success = 1
            rospy.loginfo("Success")
        else:
            success = 0
            rospy.loginfo("Failure")
        
        return success
    
    def go_to_pose_goal(self, pose_goal=None):

        move_group = self.move_group

        move_group.set_pose_target(pose_goal)
        move_success = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

        if move_success:
            success = 1
            rospy.loginfo("Success")
        else:
            success = 0
            rospy.loginfo("Failure")
        
        return success
    
    def plan_to_pose_goal(self, pose_goal=None): 
        move_group = self.move_group
        move_group.set_pose_target(pose_goal)
        plan = move_group.plan()
        move_group.stop()
        move_group.clear_pose_targets()

        if not plan[1].joint_trajectory.points:
            success = 0
            rospy.loginfo("Plan Failed")
        else:
            success = 1
            rospy.loginfo("Plan Succeed")
        
        return success
    
    def move_trajectory(self, end_pose): 
        '''
        This function execute a trajectory of from current pose to goal pose
        '''
        waypoints = []
        waypoints.append(end_pose)   
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.01, 1000.0) 
        move_success = self.move_group.execute(plan, wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        if move_success:
            success = 1
            rospy.loginfo("Success")
        else:
            success = 0
            rospy.loginfo("Failure")

        return success

    def move_gripper(self, width=0.035):

        move_group = self.move_group_hand
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = width
        joint_goal[1] = width

        move_success = move_group.go(joint_goal, wait=False)
        move_group.stop()

        if move_success:
            success = 1
            rospy.loginfo("Success")
        else:
            success = 0
            rospy.loginfo("Failure")
        
        return success

    def move_gripper2(self, width=0.04, speed = 0.05):
        self.move_action_client = actionlib.SimpleActionClient("/franka_gripper/move", franka_gripper.msg.MoveAction)
        self.move_action_client.wait_for_server()
        goal = franka_gripper.msg.MoveGoal()
        goal.width = width
        goal.speed = speed
        self.move_action_client.send_goal(goal)
        result = self.move_action_client.wait_for_result(rospy.Duration(1.5))

        return result

    def move_gripper3(self, width=0.04, max_effort = 0.1):
        self.move_action_client = actionlib.SimpleActionClient("/franka_gripper/action", GripperCommandAction)
        self.move_action_client.wait_for_server()
        goal = GripperCommandGoal()
        goal.command.position = width
        goal.command.max_effort = max_effort
        self.move_action_client.send_goal(goal)
        result = self.move_action_client.wait_for_result(rospy.Duration(15.))

        return result

    def move_gripper4(self, width=0.04, epsilon_inner = 0.005, epsilon_outer = 0.05, speed = 0.05, force = 1.0):
        self.move_action_client = actionlib.SimpleActionClient("/franka_gripper/grasp", franka_gripper.msg.GraspAction)
        self.move_action_client.wait_for_server()
        goal = franka_gripper.msg.GraspGoal()
        goal.width = width
        goal.epsilon.inner = epsilon_inner
        goal.epsilon.outer = epsilon_outer
        goal.speed = speed
        goal.force = force
        self.move_action_client.send_goal(goal)
        result = self.move_action_client.wait_for_result(rospy.Duration(15.))

        return result
    
    def stop_gripper(self): 
        self.stop_action_client = actionlib.SimpleActionClient("/franka_gripper/stop", franka_gripper.msg.StopAction)
        self.stop_action_client.wait_for_server()
        goal = franka_gripper.msg.StopGoal()
        self.stop_action_client.send_goal(goal)
        result = self.stop_action_client.wait_for_result(rospy.Duration(15.))

        return result

    def stop_movement_handle(self, req):

        move_group = self.move_group
        move_group.stop()

        return 1

    def stop_movement_handle2(self, req):

        result = self.stop_gripper()

        return result
    
    def go_to_joint_state_handle(self, req):
        joint_goal = req.goal_joint.position
        success = self.go_to_joint_state(joint_goal)

        return success

    def go_to_pose_goal_handle(self, req):
        pose_goal = req.goal_pose
        success = self.go_to_pose_goal(pose_goal)

        return success
    
    def move_traj_goal_handle(self, req):
        pose_goal = req.goal_pose
        success = self.move_trajectory(pose_goal)

        return success

    def plan_to_pose_goal_handle(self, req):
        pose_goal = req.goal_pose
        success = self.plan_to_pose_goal(pose_goal)

        return success

    def move_gripper_handle(self, req):
        width = req.width
        success = self.move_gripper2(width)

        return success
    
    def get_joints_state_handle(self, req): 
        curr_state = self.move_group.get_current_joint_values()
        return (curr_state, )

    def get_pose_handle(self, req): 
        pose = self.move_group.get_current_pose(req.link_name)
        curr_pose = [pose.pose.position.x, 
                     pose.pose.position.y, 
                     pose.pose.position.z, 
                     pose.pose.orientation.x, 
                     pose.pose.orientation.y, 
                     pose.pose.orientation.z, 
                     pose.pose.orientation.w]
        return (curr_pose, )

    def getForceCb(self, msg): 
        '''
        This function save the force 
        '''
        if msg is None: 
            rospy.logwarn('forceCb: msg is None!')
        else: 
            Force_msg = msg.wrench.force
            self.force = [Force_msg.x, Force_msg.y, Force_msg.z]
    
    def get_force_handle(self, req): 
        '''
        This function get the current force and tranfromed it into world frame: 
        '''
        pose = self.move_group.get_current_pose()
        quat = [pose.pose.orientation.x, 
                pose.pose.orientation.y, 
                pose.pose.orientation.z, 
                pose.pose.orientation.w]
        r = R.from_quat(quat)
        Rot = r.as_matrix()
        force_ee = self.force
        if len(force_ee) != 0: 
            force = Rot @ np.asarray(force_ee)
        else: 
            print('cannot receive force')
            force = np.array([])

        return (force, )

    def get_gripper_state_handle(self, req): 
        curr_gripper_state = self.move_group_hand.get_current_joint_values()
        return (curr_gripper_state, )

    def add_mesh_handle(self, req): 
        object_path = req.mesh_path
        refer_frame = req.refer_frame
        object_id = req.object_id
        object_pose_list = req.object_pose_list
        size = req.size
        S = self.add_mesh(object_path, refer_frame, object_id, object_pose_list, size)
        if S == True: 
            rospy.loginfo('the object has been added to the scene')
        else: 
            rospy.logerr('the object is failed to be added to the scene')

        objects_in_scene = self.scene.get_known_object_names()
        return (objects_in_scene, )

    def attach_mesh_handle(self, req): 
        object_path = req.mesh_path
        object_id = req.object_id
        object_pose_list = req.object_pose_list
        size = req.size
        S = self.attach_mesh(object_path, object_id, object_pose_list, size)
        if S == True: 
            rospy.loginfo('the object has been attached to the endeffector')
        else: 
            rospy.logerr('the object is failed to be attached to the endeffector')
        
        attached_object = self.scene.get_attached_objects()
        return (attached_object, )

    def detach_mesh_handle(self, req): 
        object_name = req.mesh_name
        S = self.detachMesh(object_name)

        return S

    def add_box_handle(self, req): 
        box_name = req.box_name
        refer_frame = req.refer_frame
        box_pose_list = req.box_pose_list
        box_size = req.box_size
        S = self.add_box(box_name, box_size, box_pose_list, refer_frame)
        if S == True: 
            rospy.loginfo('the box has been added to the scene')
        else: 
            rospy.logerr('the box is failed to be added to the scene')

        objects_in_scene = self.scene.get_known_object_names()
        return (objects_in_scene, )

    ## functions for planning scene: 
    def allowReplanning(self): 
        self.move_group.allow_replanning(value=True) 
        self.move_group_hand.allow_replanning(value=True)

    def add_box(self, box_name, box_size, box_pose_list, refer_frame):
        ''' 
        This function add a box into planning scene
        '''
        scene = self.scene
        box_pose = geometry_msgs.msg.PoseStamped()
        print('the reference frame for object ', box_name, ' is ', str(refer_frame))
        box_pose.pose.position.x = box_pose_list[0]
        box_pose.pose.position.y = box_pose_list[1]
        box_pose.pose.position.z = box_pose_list[2]
        box_pose.pose.orientation.x = box_pose_list[3]
        box_pose.pose.orientation.y = box_pose_list[4]
        box_pose.pose.orientation.z = box_pose_list[5]
        box_pose.pose.orientation.w = box_pose_list[6]
        box_pose.header.frame_id = refer_frame
        scene.add_box(box_name, box_pose, size=(box_size[0], box_size[1], box_size[2]))
        time.sleep(0.5)
        if box_name in self.scene.get_known_object_names(): 
            return True 
        else: 
            return False

    def add_mesh(self, object_path, refer_frame, object_id, object_pose_list, size = (1,1,1)): 
        """
        This function load object into environment from mesh file, used for planning scene construction 
        input params: 
            object_path: the mesh file of object to be loaded
            refer_frame: the reference frame of loading pose 
            object_id: the ID of the object
            object_pose: the loading pose: [Pose, Quaternion]
            size: the scale of mesh file
        """
        # check if the object is already in the scene: 
        if object_id in self.scene.get_known_object_names(): 
            self.scene.remove_world_object(object_id)
        object_pose = geometry_msgs.msg.PoseStamped()
        object_pose.header.frame_id = refer_frame
        print('the reference frame for object ', object_id, ' is ', str(refer_frame))
        object_pose.pose.position.x = object_pose_list[0]
        object_pose.pose.position.y = object_pose_list[1]
        object_pose.pose.position.z = object_pose_list[2]
        object_pose.pose.orientation.x = object_pose_list[3]
        object_pose.pose.orientation.y = object_pose_list[4]
        object_pose.pose.orientation.z = object_pose_list[5]
        object_pose.pose.orientation.w = object_pose_list[6]

        print(object_path)
        self.scene.add_mesh(object_id, object_pose, object_path, size)
        time.sleep(0.5)
        if object_id in self.scene.get_known_object_names(): 
            return True 
        else: 
            return False

    def remove_object_handle(self, req):
        '''
        This function remove the objects from scene
        ''' 
        object_id = req.object_id
        if object_id in self.scene.get_known_object_names(): 
            self.scene.remove_world_object(object_id)   
            time.sleep(0.5)
            if object_id in self.scene.get_known_object_names(): 
                print('object list is: ', self.scene.get_known_object_names())
                return False 
            else: 
                return True
        else: 
            print('object to be removed is not in the scene')
            return True

    def attach_mesh(self, object_path, object_id, object_pose_list, size = (1,1,1), refer_frame='world'): 
        '''
        This function attach a mesh into the robot end effector
        '''
        # check if the object is already in the scene: 
        if object_id in self.scene.get_known_object_names() and object_path != '': 
            self.scene.remove_world_object(object_id)
        object_pose = geometry_msgs.msg.PoseStamped()
        object_pose.header.frame_id = refer_frame
        object_pose.pose.position.x = object_pose_list[0]
        object_pose.pose.position.y = object_pose_list[1]
        object_pose.pose.position.z = object_pose_list[2]
        object_pose.pose.orientation.x = object_pose_list[3]
        object_pose.pose.orientation.y = object_pose_list[4]
        object_pose.pose.orientation.z = object_pose_list[5]
        object_pose.pose.orientation.w = object_pose_list[6]

        touch_links = self.robot.get_link_names('panda_hand') # disable collsion detection between objact and robot hand
        eef_link = self.eef_link
        self.scene.attach_mesh(eef_link, object_id, pose = object_pose, filename = object_path, size = size, touch_links = touch_links)
        rospy.sleep(1)

        attached_object = self.scene.get_attached_objects()
        if object_id in attached_object: 
            return True
        else: 
            return False

    def detachMesh(self, name): 
        '''
        This function remove the attached mesh model
        '''
        eef_link = self.eef_link
        self.scene.remove_attached_object(eef_link, name)
        self.scene.remove_world_object(name)
        time.sleep(0.5)

        attached_object = self.scene.get_attached_objects()
        if name in attached_object: 
            return False
        else: 
            return True

    def addGround(self): 
        '''
        This function loads only ground to the planning scene
        '''
        ground_name = 'Ground'
        ground_size = [10, 10, 0.01]
        ground_pose = [0, 0, -0.01, 0, 0, 0, 1]
        refer_frame = 'world' #self.robot.get_planning_frame()
        r1 = self.add_box(ground_name, ground_size, ground_pose, refer_frame)
        if r1 == True: 
            print('-----ground has been loaded to scene--------')
        return r1

    def run_panda_service(self):
        rospy.Service("panda_move_to_joint_state_server", move2joint, self.go_to_joint_state_handle)
        rospy.Service("panda_move_to_pose_server", move2pose, self.go_to_pose_goal_handle)
        rospy.Service("panda_plan_to_pose_server", move2pose, self.plan_to_pose_goal_handle)
        rospy.Service("panda_move_to_traj_server", move2pose, self.move_traj_goal_handle)
        rospy.Service("panda_move_gripper_server", moveGripper, self.move_gripper_handle)
        rospy.Service("panda_stop_server", stop, self.stop_movement_handle2)
        rospy.Service("panda_get_states", getJoints, self.get_joints_state_handle)
        rospy.Service("panda_get_pose", getPose, self.get_pose_handle)      
        rospy.Service("panda_get_gripper", getGripper, self.get_gripper_state_handle)
        rospy.Service("panda_get_force", getForce, self.get_force_handle)
        rospy.Service("add_mesh", addMesh, self.add_mesh_handle)
        rospy.Service("add_box", addBox, self.add_box_handle)
        rospy.Service("attach_mesh", attachMesh, self.attach_mesh_handle)
        rospy.Service("detach_mesh", removeMesh, self.detach_mesh_handle)
        rospy.Service("remove_object", removeObject, self.remove_object_handle)



