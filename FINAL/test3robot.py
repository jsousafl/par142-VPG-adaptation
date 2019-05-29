#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 27 17:10:55 2018

@author: jsousafl
"""

from robot import Robot
import numpy as np
import math
import utilsedit

tcp_host_ip = '192.168.1.5' # IP and port to robot arm as TCP client (UR5)
tcp_port = 30002
rtc_host_ip = '100.127.7.223' # IP and port to robot arm as real-time client (UR5)
rtc_port = 30003
workspace_limits = np.asarray([[-0.112, 0.112], [0.16, 0.384], [0.0, 0.12]]) # Cols: min max, Rows: x y z (define workspace limits in robot coordinates)
#calib_grid_step = 0.05
#checkerboard_offset_from_tool = [0,-0.13,0.02]
tool_orientation = [1.7453503794018523, -2.6120141394306633, 0.0]
#[-1.82, 0.72, 1.57] # [0,-2.22,2.22] # [2.22,2.22,0]
#tool_position = [0.05,0.35,0.1]
calib_grid_stepz = 0.1

tool_position = utilsedit.altern2cart(np.transpose([workspace_limits[0][1],workspace_limits[1][0],workspace_limits[2][1]]))

#x_initial = np.zeros(num_step_face2+1)
#y_initial = np.zeros(num_step_face2+1)
#x = np.zeros(num_step_face2+1*num_step_face1+1)
#y = np.zeros(num_step_face2+1*num_step_face1+1)
#for i in range(num_step_face2+1):
#    x_initial[i] = tool_position[0] + i*step_face2
#    y_initial[i] = a_perpendicular*x_initial[i] + b_perpendicular
#    b = y_initial[i]-a*x_initial[i]
#    for j in range(num_step_face1+1):
#        x[i+j] = x_initial[i] + j*step_face1
#        y[i+j] = a*x[i+j] + b

#calib_grid_pts = np.concatenate((x, y), axis=1)
#print(x)
#print(y)     
#print(calib_grid_pts)    

robot = Robot(False, None, None, workspace_limits,
              tcp_host_ip, tcp_port, rtc_host_ip, rtc_port,
              False, None, None)


robot.move_to(tool_position,tool_orientation)
