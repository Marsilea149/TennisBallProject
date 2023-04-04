#!/usr/bin/env python
# -*- coding: utf-8 -*-

#Libraries

from adafruit_servokit import ServoKit    #https://circuitpython.readthedocs.io/projects/servokit/en/latest/
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

# move the servo with input angles  
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


if __name__ == '__main__':
    init()
    time.sleep(2)

    #Define the neutral angles for each joint
    #At these angle the robot arm is athe the axis x for each joint
    base_neutral = 90.0
    shoulder_neutral = 115.0
    elbow_neutral = 110.0

    i = 0
    delta = 1
    last_angle = pca.servo[i].angle
    last_time = time.time()
    dt = 0.001
    while True:

        if pca.servo[i].angle > 160:
            delta = -1
        elif pca.servo[i].angle < 20:
            delta = 1

        pca.servo[i].angle += delta
        time.sleep(dt)
        print("base angle: ", pca.servo[i].angle)

        elapsed_time = time.time()-last_time
        last_time = time.time()
        speed = (pca.servo[i].angle - last_angle)/elapsed_time
        print("speed deg/s: ", speed)
        last_angle = pca.servo[i].angle
        
    