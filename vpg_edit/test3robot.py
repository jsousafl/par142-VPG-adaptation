#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 27 17:10:55 2018

@author: jsousafl
"""

from robot import Robot
import numpy as np
import math

tcp_host_ip = '192.168.1.5' # IP and port to robot arm as TCP client (UR5)
tcp_port = 30002
rtc_host_ip = '100.127.7.223' # IP and port to robot arm as real-time client (UR5)
rtc_port = 30003
workspace_limits = np.asarray([[0.05, 0.177], [0.35, 0.49], [0.1, 0.2]]) # Cols: min max, Rows: x y z (define workspace limits in robot coordinates)
#calib_grid_step = 0.05
#checkerboard_offset_from_tool = [0,-0.13,0.02]
tool_orientation = [-1.6365310622765092, 1.563631616437465, 2.0273420502040866]
#[-1.82, 0.72, 1.57] # [0,-2.22,2.22] # [2.22,2.22,0]
tool_position = [0.05,0.35,0.1]
calib_grid_stepz = 0.1

# TESTE PARA A RETA
a = math.tan(math.pi/2-math.asin(8/12))
num_step_face1 = 3
step_face1 = (0.177-tool_position[0])/num_step_face1
   
gridspace_z = np.linspace(workspace_limits[2][0], workspace_limits[2][1], 1 + (workspace_limits[2][1] - workspace_limits[2][0])/calib_grid_stepz)

a_perpendicular = math.tan(138.2/180*math.pi)
b_perpendicular = tool_position[1]-a_perpendicular*tool_position[0]
num_step_face2 = 3
step_face2 = (0.25-tool_position[0])/num_step_face2

x_initial = np.zeros(num_step_face2+1)
y_initial = np.zeros(num_step_face2+1)
x = np.array([])
y = np.array([])
for i in range(num_step_face2+1):
    x_initial[i] = tool_position[0] + i*step_face2
    y_initial[i] = a_perpendicular*x_initial[i] + b_perpendicular
    b = y_initial[i]-a*x_initial[i]
    for j in range(num_step_face1+1):
        x = np.append(x,[x_initial[i] + j*step_face1])
        y = np.append(y,[a*x[-1] + b])

z = np.array([])
for i in range((num_step_face2+1)*(num_step_face1+1)):
    z = np.append(z,gridspace_z)
x = np.split(x,(num_step_face2+1)*(num_step_face1+1))
y = np.split(y,(num_step_face2+1)*(num_step_face1+1))
z = np.split(z,len(z))

calib_grid_x, calib_grid_z = np.meshgrid(x, gridspace_z)
calib_grid_y, calib_grid_z = np.meshgrid(y, gridspace_z)
num_calib_grid_pts = calib_grid_x.shape[0]*calib_grid_x.shape[1]
calib_grid_x.shape = (num_calib_grid_pts,1)
calib_grid_y.shape = (num_calib_grid_pts,1)
print(x)
print(y)
calib_grid_pts = np.concatenate((calib_grid_x, calib_grid_y, z), axis=1)

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

"""
for calib_pt_idx in range(num_calib_grid_pts):
    tool_position = calib_grid_pts[calib_pt_idx,:]
    print("Position")
    print(tool_position)
    print("Orientation")
    print(tool_orientation)
    #robot.move_to(tool_position, tool_orientation)
    time.sleep(1)
""" 
"""actual_tool_pose = robot.get_urx_data('cartesian_info')
actual_tool_pose[2] = actual_tool_pose[2] + 0.1
print(actual_tool_pose)
actual_tool_pose = np.array(actual_tool_pose)
robot.move_to(actual_tool_pose[:3],actual_tool_pose[3:])

joint_config2 = [-np.pi, -np.pi/2, np.pi/2, 0, np.pi/2, np.pi]
for i in range(2):
   robot.move_joints(joint_config) 
   robot.move_joints(joint_config2)"""

robot.move_to(tool_position,tool_orientation)
