#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time    
import numpy as np
import math
import servo_control as sc
import fk as fk
import ik as ik

if __name__ == '__main__':
    sc.init()
    
    #######################Go to down position###########################
    #input joint angles in joint frame in degree
    #range: -80 (robot's right)=>80 (robot's left)
    base_angle = 0.0
    #range: 70(up)=>-30(down)
    shoulder_angle = 30.0
    #range: 70(up)=>-20(down)
    elbow_angle = -60.0
    
    sc.servo_control(base_angle, shoulder_angle, elbow_angle, False)
    time.sleep(3)
    print(fk.compute_end_effector_pose(base_angle, shoulder_angle, elbow_angle))

    #######################Close the gripper###########################
    sc.servo_control(base_angle, shoulder_angle, elbow_angle, True)
    time.sleep(1)
    #fk.compute_end_effector_pose(base_angle, shoulder_angle, elbow_angle)
    
    #######################Go to target###########################
    #target end effector pose in global frame
    target_1 = np.array([[0.18],
                       [0.0],
                       [0.4]])
    joint_angles = np.array([[base_angle],
                             [shoulder_angle],
                             [elbow_angle]])
    # send command to reach the target
    joint_angles = ik.go_to_target(target_1, joint_angles)
    print("===end of ik angles: ")
    print(joint_angles)
    time.sleep(1)
    #######################Open the gripper###############################
    sc.servo_control(joint_angles[0][0], joint_angles[1][0], joint_angles[2][0], True)
    
    