#!/usr/bin/env python
# -*- coding: utf-8 -*-

#Libraries

from adafruit_servokit import ServoKit    #https://circuitpython.readthedocs.io/projects/servokit/en/latest/
import math
import time

#Constants used to set the nomber of servos in use
nbPCAServo=4

#Parameters used to map between pwm pulse (ms) and servo angles (degree)
MIN_IMP  =[500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500]
MAX_IMP  =[2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500]
MIN_ANG  =[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
MAX_ANG  =[180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180]

#Objects
pca = ServoKit(channels=16)
pca.frequency = 50

# initialize the servos
def init():
    for i in range(nbPCAServo):
        pca.servo[i].set_pulse_width_range(MIN_IMP[i] , MAX_IMP[i])


"""
Move the servo with input angle commands, the input angles are relative to the neutral 
positions of each joint.

@param base_angle: base joint target angle in degree
@param shoulder_angle: shoulder joint target angle in degree
@param elbow_angle: elbow joint target angle in degree
@param close: close the gripper
Example:
    #input joint angles in joint frame in degree
    #range: -80 (robot's right)=>80 (robot's left)
    base_angle = 0.0
    #range: 70(up)=>-30(down)
    shoulder_angle = 30.0
    #range: 70(up)=>-20(down)
    elbow_angle = -60.0

    servo_control(base_angle=30.0, shoulder_angle=35, elbow_angle=-10, close = True)
"""
def servo_control(base_angle, shoulder_angle, elbow_angle, close = True):
    #Define the neutral angles for each joint
    #At these angle the robot arm is athe the axis x for each joint
    base_neutral = 90.0
    shoulder_neutral = 115.0
    elbow_neutral = 110.0
    
    # Take into account the servo offset with 'neutral' values of each servo 
    #base: 10->170  neutral=90
    pca.servo[0].angle = base_angle + base_neutral
    #shoulder: 45->145  neutral=115
    pca.servo[1].angle = shoulder_neutral - shoulder_angle 
    #elbow: 60->150   neutral=110
    pca.servo[2].angle = elbow_neutral - elbow_angle

    # pca.servo[3].angle = 85
    # print(pca.servo[3].angle)
    # #gripper:85=open 122=close
    if close == True:
       pca.servo[3].angle = 128
       print("close: ", pca.servo[3].angle)
    else:
       pca.servo[3].angle = 78
       print("open: ", pca.servo[3].angle)

def sign(a):
    return (a > 0) - (a < 0)
    
def move_servo_absolute(target_base_angle = 90.0, 
                        target_shoulder_angle = 115.0, 
                        target_elbow_angle = 110.0):

    dt = 0.001
    # i = 0 

    delta_base = sign(target_base_angle-pca.servo[0].angle) 
    delta_shoulder = sign(target_shoulder_angle-pca.servo[1].angle) 
    delta_elbow = sign(target_elbow_angle-pca.servo[2].angle)
    
    #acceptable error
    ae = 1.0
    while abs(pca.servo[0].angle - target_base_angle) > ae or abs(pca.servo[1].angle - target_shoulder_angle) > ae or abs(pca.servo[2].angle - target_elbow_angle) > ae : 
        
        if abs(pca.servo[0].angle - target_base_angle) < ae:
            delta_base = 0.0
        if abs(pca.servo[1].angle - target_shoulder_angle) < ae:
            delta_shoulder = 0.0
        if abs(pca.servo[2].angle - target_elbow_angle) < ae:
            delta_elbow = 0.0
        
        pca.servo[0].angle += delta_base
        # print("---base angle: ", pca.servo[0].angle)
        pca.servo[1].angle += 2*delta_shoulder
        # print("---shoulder angle: ", pca.servo[1].angle)
        pca.servo[2].angle += 2*delta_elbow
        # print("---elbow angle: ", pca.servo[2].angle)
        
        time.sleep(dt)

    # print("base error: ", abs(pca.servo[0].angle - target_base_angle))
    # print("shoulder error: ", abs(pca.servo[1].angle - target_shoulder_angle))
    # print("elbow error: ", abs(pca.servo[2].angle - target_elbow_angle))

    # delta_base = sign(target_base_angle-pca.servo[i].angle) 
    # while abs(pca.servo[i].angle - target_base_angle) > ae: 
    #     pca.servo[i].angle += delta_base
    #     time.sleep(dt)
    #     print("base angle: ", pca.servo[i].angle)