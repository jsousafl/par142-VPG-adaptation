#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov 22 15:21:10 2018

@author: jsousafl
"""

import urx

"""pose1 = [0.3, 0.15, 0.1]
orientation = [3.0,-1.0,0.0]
pose1.extend(orientation)
rob = urx.Robot("192.168.1.5")
pose2 = [0.3, 0.15, 0.1]
orientation = [3.0,-1.0,0.0]
pose2.extend(orientation)
pose3 = [0.3, 0.15, 0.1]
orientation = [3.0,-1.0,0.0]
pose3.extend(orientation)"""
rob = urx.Robot("192.168.1.5")
#rob.movex('movel',pose1,0.5,0.5)
#rob.movex('movel',pose2,0.5,0.5)
#rob.movex('movel',pose3,0.5,0.5)
"""tool_analog_input2 = rob.get_analog_in(3,wait=True)
print(tool_analog_input2)
rob.close()"""

from robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper

robotiqgrip = Robotiq_Two_Finger_Gripper(rob)
robotiqgrip.open_gripper()
robotiqgrip.close_gripper()
#rob.send_program(robotiqgrip.ret_program_to_run())
#status = robotiqgrip.get_status_obj()
print('Alo')
#print(status)
robotiqgrip.open_gripper()
#status = robotiqgrip.get_status_obj()
print('Alo')
#print(status)
rob.close()