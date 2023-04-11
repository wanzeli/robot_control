#!/usr/bin/env python3
# Utility functions
# Author: Hongtao Wu
# Johns Hopkins University

import numpy as np
import yaml
from yaml import CLoader
from scipy.spatial.transform import Rotation as R
import open3d as o3d
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint



def quat2rotm(quat):
    """
    Quaternion to rotation matrix.
    
    Args:
    - quat (4, numpy array): quaternion w, x, y, z
    Returns:
    - rotm: (3x3 numpy array): rotation matrix
    """
    w = quat[0]
    x = quat[1]
    y = quat[2]
    z = quat[3]

    s = w*w + x*x + y*y + z*z

    rotm = np.array([[1-2*(y*y+z*z)/s, 2*(x*y-z*w)/s,   2*(x*z+y*w)/s  ],
                     [2*(x*y+z*w)/s,   1-2*(x*x+z*z)/s, 2*(y*z-x*w)/s  ],
                     [2*(x*z-y*w)/s,   2*(y*z+x*w)/s,   1-2*(x*x+y*y)/s]
    ])

    return rotm

def getHomogeneous(pose): 
    '''
    get the homogeneous of transformation, 
    pose: 0 - 2: translation of x y z
    pose: 3 - 6: Quaternion [x, y, z, w]
    '''
    q_R = pose[3:]
    r = R.from_quat(q_R)
    Rm = r.as_matrix()
    pw = np.array([[pose[0], pose[1], pose[2]]])
    T = np.append(Rm, pw.T, axis=1)
    T = np.append(T, np.array([[0, 0, 0, 1]]), axis = 0)
    T = np.asarray(T)

    return T

def make_rigid_transformation(pos, rotm):
    """
    Rigid transformation from position and orientation.
    Args:
    - pos (3, numpy array): translation
    - rotm (3x3 numpy array): orientation in rotation matrix
    Returns:
    - homo_mat (4x4 numpy array): homogenenous transformation matrix
    """
    homo_mat = np.c_[rotm, np.reshape(pos, (3, 1))]
    homo_mat = np.r_[homo_mat, [[0, 0, 0, 1]]]
    
    return homo_mat

def pose_inv(pose):
    """
    Inverse of a homogenenous transformation.
    Args:
    - pose (4x4 numpy array)
    Return:
    - inv_pose (4x4 numpy array)
    """
    R = pose[:3, :3]
    t = pose[:3, 3]

    inv_R = R.T
    inv_t = - np.dot(inv_R, t)

    inv_pose = np.c_[inv_R, np.transpose(inv_t)]
    inv_pose = np.r_[inv_pose, [[0, 0, 0, 1]]]

    return inv_pose


def get_mat_log(R):
  """
  Get the log(R) of the rotation matrix R.
  
  Args:
  - R (3x3 numpy array): rotation matrix
  Returns:
  - w (3, numpy array): log(R)
  """
  theta = np.arccos((np.trace(R) - 1) / 2)
  w_hat = (R - R.T) * theta / (2 * np.sin(theta))  # Skew symmetric matrix
  w = np.array([w_hat[2, 1], w_hat[0, 2], w_hat[1, 0]])  # [w1, w2, w3]

  return w


def rotm2quat(R):
    """
    Get the quaternion from rotation matrix.
    
    Args:
    - R (3x3 numpy array): rotation matrix
    Return:
    - q (4, numpy array): quaternion, w, x, y, z
    """
    w = get_mat_log(R)
    theta = np.linalg.norm(w)

    if theta < 0.001:
        q = np.array([0, 0, 0, 1])
        return q

    axis = w / theta

    q = np.sin(theta/2) * axis
    q = np.r_[np.cos(theta/2), q]

    return q

def rotm2angle(R):
    # From: euclideanspace.com

    epsilon = 0.01 # Margin to allow for rounding errors
    epsilon2 = 0.1 # Margin to distinguish between 0 and 180 degrees

    assert(isRotm(R))

    if ((abs(R[0][1]-R[1][0])< epsilon) and (abs(R[0][2]-R[2][0])< epsilon) and (abs(R[1][2]-R[2][1])< epsilon)):
        # Singularity found
        # First check for identity matrix which must have +1 for all terms in leading diagonaland zero in other terms
        if ((abs(R[0][1]+R[1][0]) < epsilon2) and (abs(R[0][2]+R[2][0]) < epsilon2) and (abs(R[1][2]+R[2][1]) < epsilon2) and (abs(R[0][0]+R[1][1]+R[2][2]-3) < epsilon2)):
            # this singularity is identity matrix so angle = 0
            return [0,1,0,0] # zero angle, arbitrary axis

        # Otherwise this singularity is angle = 180
        angle = np.pi
        xx = (R[0][0]+1)/2
        yy = (R[1][1]+1)/2
        zz = (R[2][2]+1)/2
        xy = (R[0][1]+R[1][0])/4
        xz = (R[0][2]+R[2][0])/4
        yz = (R[1][2]+R[2][1])/4
        if ((xx > yy) and (xx > zz)): # R[0][0] is the largest diagonal term
            if (xx< epsilon):
                x = 0
                y = 0.7071
                z = 0.7071
            else:
                x = np.sqrt(xx)
                y = xy/x
                z = xz/x
        elif (yy > zz): # R[1][1] is the largest diagonal term
            if (yy< epsilon):
                x = 0.7071
                y = 0
                z = 0.7071
            else:
                y = np.sqrt(yy)
                x = xy/y
                z = yz/y
        else: # R[2][2] is the largest diagonal term so base result on this
            if (zz< epsilon):
                x = 0.7071
                y = 0.7071
                z = 0
            else:
                z = np.sqrt(zz)
                x = xz/z
                y = yz/z
        return [angle,x,y,z] # Return 180 deg rotation

    # As we have reached here there are no singularities so we can handle normally
    s = np.sqrt((R[2][1] - R[1][2])*(R[2][1] - R[1][2]) + (R[0][2] - R[2][0])*(R[0][2] - R[2][0]) + (R[1][0] - R[0][1])*(R[1][0] - R[0][1])) # used to normalise
    if (abs(s) < 0.001):
        s = 1 

    # Prevent divide by zero, should not happen if matrix is orthogonal and should be
    # Caught by singularity test above, but I've left it in just in case
    angle = np.arccos(( R[0][0] + R[1][1] + R[2][2] - 1)/2)
    x = (R[2][1] - R[1][2])/s
    y = (R[0][2] - R[2][0])/s
    z = (R[1][0] - R[0][1])/s

    #aa = angle * np.array([x, y, z])
    aa = [angle, x, y, z]

    return aa

def isRotm(R) :
    # Checks if a matrix is a valid rotation matrix.
    # Forked from Andy Zeng
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def dist_to_convhull(points, center, v): 
    '''
    This function calculates the distance from hanging center to 
    the convex hull of point cloud along direction v
    '''
    pc = o3d.geometry.PointCloud()
    v3d = o3d.utility.Vector3dVector
    pc.points = v3d(points)
    hull, _ = pc.compute_convex_hull()
    convex_hull = o3d.t.geometry.TriangleMesh.from_legacy(hull)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(convex_hull) 
    ray = [[center[0], center[1], center[2], v[0], v[1], v[2]]]
    ray_tensor = o3d.core.Tensor(ray, dtype=o3d.core.Dtype.Float32)
    ans = scene.cast_rays(ray_tensor)
    d = ans['t_hit'].numpy()

    return d[0]

def Preprocess(supporter_points, cam_pose, num_neighbors=50, std_ratio=2.0): 
    if len(supporter_points) > 100: 
        pt = o3d.geometry.PointCloud()
        pt.points = o3d.utility.Vector3dVector(supporter_points)

        pt.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.005, max_nn=20))
        supporter_normal_vectors = np.asarray(pt.normals)
        supporter_points = np.asarray(pt.points)
        # sor_pcd, ind = pt.remove_statistical_outlier(num_neighbors, std_ratio)
        # print(len(np.asarray(sor_pcd.points)))

        # sor_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.005, max_nn=20))
        # supporter_normal_vectors = np.asarray(sor_pcd.normals)
        # supporter_points = np.asarray(sor_pcd.points)

        # Adjust the normal vectors directions based on camera poses: 
        for l in range(len(supporter_normal_vectors)): 
            cam_position = np.array([cam_pose[0][3], cam_pose[1][3], cam_pose[2][3]])
            p_i = supporter_points[l]
            v = cam_position - p_i
            if np.dot(supporter_normal_vectors[l], v) < 0:
                supporter_normal_vectors[l] = -supporter_normal_vectors[l]
    else: 
        supporter_points = []
        supporter_normal_vectors = []
        print('no enough points in the depth image')
    return supporter_points, supporter_normal_vectors

def saveply(points, ply_name): 
    '''
    THis function save points as ply file: 
    '''
    pt = o3d.geometry.PointCloud()
    pt.points = o3d.utility.Vector3dVector(points)
    o3d.io.write_point_cloud(ply_name, pt)

def pc_filt(points, num_neighbors = 50, std_ratio = 1.0): 
    '''
    This function remove noise from point cloud with k-nearest filting
    '''
    pt = o3d.geometry.PointCloud()
    pt.points = o3d.utility.Vector3dVector(points)    
    sor_pcd, ind = pt.remove_statistical_outlier(num_neighbors, std_ratio)
    points_f = np.asarray(sor_pcd.points)

    return points_f, ind

def depthToPointCloud(cam_param, depth_img, T, filt_threshold = 0.5): 
    '''
    This function transforms the depth image to point cloud: 
    cam_param: camera parameters, [f_length_x, f_length_y, o_x, o_y]
    depth_img: the depth image
    T: camera pose matrix
    '''
    intrinsic_matrix = np.array([[cam_param[0], 0, cam_param[2]], 
                                    [0, cam_param[1], cam_param[3]], 
                                    [0, 0, 1]]) # intrinsic matrix of the camera
    near_depth = depth_img[np.where(depth_img < filt_threshold)] # the depth value smaller than filt_threshold
    near_depth_matrix = np.asarray([near_depth, near_depth, near_depth]) # concatenate the depth value into a matrix for matrix multiply
    near_x_indexes = np.where(depth_img < filt_threshold)[1] # coordinates in the image that is near to the target
    near_y_indexes = np.where(depth_img < filt_threshold)[0]
    P_img = np.asarray([near_x_indexes, near_y_indexes, np.ones(len(near_y_indexes))])
    P_cam = np.multiply(near_depth_matrix, np.dot(np.linalg.inv(intrinsic_matrix), P_img)) # Depth*T-1*P_img, get points in camera frame
    b = np.asarray([np.ones(len(P_cam[0]))])
    P_cam_c = np.r_[P_cam, b]
    point_cloud = (np.dot(T, P_cam_c)).T
    point_cloud = point_cloud[:, 0:3]
    return point_cloud

def scale_trajectory_speed(traj, scale):
    # Create a new trajectory object
    new_traj = RobotTrajectory()
    # Initialize the new trajectory to be the same as the planned trajectory
    new_traj.joint_trajectory = traj.joint_trajectory
    # Get the number of joints involved
    n_joints = len(traj.joint_trajectory.joint_names)
    # Get the number of points on the trajectory
    n_points = len(traj.joint_trajectory.points)
    # Store the trajectory points
    points = list(traj.joint_trajectory.points)
    # Cycle through all points and scale the time from start, speed and acceleration
    for i in range(n_points):
        point = JointTrajectoryPoint()
        point.time_from_start = traj.joint_trajectory.points[i].time_from_start / scale
        point.velocities = list(traj.joint_trajectory.points[i].velocities)
        point.accelerations = list(traj.joint_trajectory.points[i].accelerations)
        point.positions = traj.joint_trajectory.points[i].positions
        for j in range(n_joints):
            point.velocities[j] = point.velocities[j] * scale
            point.accelerations[j] = point.accelerations[j] * scale * scale
        points[i] = point
    # Assign the modified points to the new trajectory
    new_traj.joint_trajectory.points = points
    # Return the new trajectory
    return new_traj