#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Open and close gripper test using library urx

import urx

rob = urx.Robot("192.168.1.5")

from robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper

robotiqgrip = Robotiq_Two_Finger_Gripper(rob)
print('Open gripper')
robotiqgrip.open_gripper()
print('Close gripper')
robotiqgrip.close_gripper()
print('Open gripper')
robotiqgrip.open_gripper()
print('Close gripper')
rob.close()