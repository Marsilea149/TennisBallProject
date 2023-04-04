#!/usr/bin/env python
# -*- coding: utf-8 -*-
import servo_control as sc


if __name__ == '__main__':
    sc.init()
    sc.servo_control(0.0, 30.0, -40.0, False)

    sc.move_servo_absolute(target_base_angle = 80.0, target_shoulder_angle = 130.0)
    print("end")
    