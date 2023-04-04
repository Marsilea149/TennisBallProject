# -*- coding: utf-8 -*-
"""
Created on Mon Dec  6 11:59:52 2021

@author: Marsi
"""
# for matrices
import numpy as np 
# for radians
import math

def axis_angle_rot_matrix(k,q):
    """
    Creates a 3x3 rotation matrix in 3D space from an axis and an angle.
 
    Input
    :param k: A 3 element array containing the unit axis to rotate around (kx,ky,kz) 
    :param q: The angle (in radians) to rotate by
 
    Output
    :return: A 3x3 element matix containing the rotation matrix
    
    Example: 
        compute a rotational matrix around z-axis of 30 degree:
        rot1 = axis_angle_rot_matrix([0,0,1], np.pi/6)
    """
     
    #15 pts 
    c_theta = np.cos(q)
    s_theta = np.sin(q)
    v_theta = 1 - np.cos(q)
    kx = k[0]
    ky = k[1]
    kz = k[2]   
     
    # First row of the rotation matrix
    r00 = kx * kx * v_theta + c_theta
    r01 = kx * ky * v_theta - kz * s_theta
    r02 = kx * kz * v_theta + ky * s_theta
     
    # Second row of the rotation matrix
    r10 = kx * ky * v_theta + kz * s_theta
    r11 = ky * ky * v_theta + c_theta
    r12 = ky * kz * v_theta - kx * s_theta
     
    # Third row of the rotation matrix
    r20 = kx * kz * v_theta - ky * s_theta
    r21 = ky * kz * v_theta + kx * s_theta
    r22 = kz * kz * v_theta + c_theta
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix

def create_homogeneous_matrix(rot, trans):
    '''
    Create an homogeneous matrix from a rotational matrix and a translational vector
    
    Example:
        k = [0,0,1]
        q = np.pi/6
        rot1 = axis_angle_rot_matrix(k, q)
        trans_z =np.array([[0],
                           [0],
                           [1]])
        h0 = create_homogeneous_matrix(rot1, trans_z)
        
        Result:
        h0 = [[ 0.8660254 -0.5        0.         0.       ]
             [ 0.5        0.8660254  0.         0.       ]
             [ 0.         0.         1.         1.       ]
             [ 0.         0.         0.         1.       ]]
    '''
    # Create the homogeneous transformation matrix
    homgen_mat = np.concatenate((rot, trans), axis=1) # side by side
 
    # Row vector for bottom of homogeneous transformation matrix
    extra_row_homgen = np.array([[0, 0, 0, 1]])
 
    # Add extra row to homogeneous transformation matrix    
    homgen_mat = np.concatenate((homgen_mat, extra_row_homgen), axis=0) # one above the other
    
    return homgen_mat


def hd_matrix(theta, d, a, alpha):
    '''
    This method computes the transformation matrix from joint i-1 to joint i.
    It uses D-H parameters:
        T(i-1)(i) = Rot_z(theta_i)*Trans_z(d_i)*Trans_x(a_i)*Rot_x(alpha_i)
        For more explanation: https://www.youtube.com/watch?v=JBNmPq8eg8w
        
    Input
    :param theta: rotational angle around z_(i-1)
    :param d: displacement along z_(i-1)
    :param a: displacement along x_i
    :param alpha: rotational angle around x_i to get z_(i-1) to z_i
    
    Output
    :return: A 4x4 Homogenous representation matrix
    
    Example: 

    '''
    
     # Store the rotation matrix around z-axis
    rot_z = axis_angle_rot_matrix([0,0,1], theta)
    zero = np.array([[0],
                    [0],
                    [0]])
    h_rot_z = create_homogeneous_matrix(rot_z, zero)
    #print("h_rot_z: ")
    #print(h_rot_z)
    
    # Store the translation along z-axis from frame i-1 to frame i
    trans_z = np.array([[0],
                        [0],
                        [d]])
    identity = np.identity(3)
    h_trans_z = create_homogeneous_matrix(identity, trans_z)
    #print("h_trans_z: ")
    #print(h_trans_z)
    
    # Store the translation matrix along x-axis
    trans_x = np.array([[a],
                        [0],
                        [0]])
    h_trans_x = create_homogeneous_matrix(identity, trans_x)
    #print("h_trans_x: ")
    #print(h_trans_x)
    
    # Store the rotational matrix around x-axis
    rot_x = axis_angle_rot_matrix([1,0,0], alpha)
    h_rot_x = create_homogeneous_matrix(rot_x, zero)
    #print("h_rot_x: ")
    #print(h_rot_x)

    # Compute the transformation matrix
    homMat = h_rot_z @ h_trans_z @ h_trans_x  @ h_rot_x
    #print(homMat)
    return homMat
   
def fk(q, d, a, alpha, p):
    n = len(q)
    
    # The position of this joint described by the index
    p_eff_x = p[0]
    p_eff_y = p[1]
    p_eff_z = p[2]
    joint_position_in_previous_frame = np.array([[p_eff_x],
                                    [p_eff_y],
                                    [p_eff_z],
                                    [1]])
    
    for i in range(n-1, -1, -1):
        #print("===========joint index: ", i)
        joint_position_in_previous_frame = hd_matrix(q[i], d[i], a[i], alpha[i]) @ joint_position_in_previous_frame
        #print("===========joint position in (i-1)th frame: ")
        #print(joint_position_in_previous_frame)
        
    return joint_position_in_previous_frame
            
    
def compute_end_effector_pose(base_angle, shoulder_angle, elbow_angle):
    # vector of rotational angles theta1, theta2, theta3
    # angle of rotation of joint 1 wrt basis
    q1 = math.radians(base_angle)
    q2 = math.radians(shoulder_angle)
    q3 = math.radians(elbow_angle)
    
    # the length of each segment in m
    a1 = 0.1862
    a2 = 0.1602
    a3 = 0.030+0.0165
    x4 = 0.100
    y4 = -0.005
    #end effector pose in the frame N-1
    eff = [x4,y4,0]
    
    # D-H parameters
    q = [q1, q2, q3]
    d = [a1, 0, 0]
    a = [0, a2, a3] #r in notebook
    alpha = [np.pi/2, 0, 0]
    
    # apply forward kinematics
    eff_after_tf = fk(q, d, a, alpha, eff)
    #print("result : ")
    #print(eff_after_tf)
    #print("======================")
    
    eff_pos = np.array([[eff_after_tf[0][0]],
                        [eff_after_tf[1][0]],
                        [eff_after_tf[2][0]]])
    return eff_pos;

'''
if __name__ == '__main__':
    print(compute_end_effector_pose(10, 30, 30))
'''