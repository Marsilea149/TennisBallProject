#!/usr/bin/env python
# -*- coding: utf-8 -*-
import servo_control as sc


if __name__ == '__main__':
    sc.init()
    #sc.servo_control(0.0, 30.0, -40.0, False)

    # Face to robot view: Base Left
    sc.move_servo_absolute(target_base_angle = 10.0, target_shoulder_angle = 85.0)
    # Face to robot view: Base Right
    sc.move_servo_absolute(target_base_angle = 170.0, target_shoulder_angle = 85.0)
    # Face to robot view: Shoulder Up
    sc.move_servo_absolute(target_base_angle = 90.0, target_shoulder_angle = 30.0)
    # Face to robot view: Shoulder Down
    sc.move_servo_absolute(target_base_angle = 90.0, target_shoulder_angle = 130.0)
    # Face to robot view: Elbow Up
    sc.move_servo_absolute(target_base_angle = 90.0, target_shoulder_angle = 100.0, target_elbow_angle = 80.0)
    # Face to robot view: Elbow Down
    sc.move_servo_absolute(target_base_angle = 90.0, target_shoulder_angle = 100.0, target_elbow_angle = 170.0)

    print("end")
    