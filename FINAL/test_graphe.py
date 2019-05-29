#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import time
import cv2
import utilsedit
from real.camera import Camera
from robot import Robot
from graphe import Graphe


    

# User options (change me)
# --------------- Setup options ---------------
tcp_host_ip = '192.168.1.5' # IP and port to robot arm as TCP client (UR3)
tcp_port = 30002
rtc_host_ip = '100.127.7.223' # IP and port to robot arm as real-time client (UR3)
rtc_port = 30003
workspace_limits = np.asarray([[-0.112, 0.112], [0.16, 0.384], [0.0, 0.13]]) # Cols: min max, Rows: u v z (define workspace limits in alternative coordinates)
tool_orientation = [1.2013852595810366, -2.898887574806905, -0.03504418257088692]
heightmap_resolution = 0.00175
# ---------------------------------------------

# Initializing robot
robot = Robot(False, None, None, workspace_limits,
              tcp_host_ip, tcp_port, rtc_host_ip, rtc_port,
              False, None, None)
robot.joint_acc = 1.4
robot.joint_vel = 1.05

position_start = np.array([-0.063,0.272,0.00])
position_end = np.array([0.1,-0.148,0.08])
position_fl =  np.array([0.33,0.22,0.0])
#robot.graph_move_to(position_start,tool_orientation)

## Get latest RGB-D image
color_img, depth_img = robot.get_camera_data()
depth_img = depth_img * robot.cam_depth_scale # Apply depth scale from calibration

# Get heightmap from RGB-D image (by re-projecting 3D point cloud)
color_heightmap, depth_heightmap = utilsedit.get_heightmap(color_img, depth_img, robot.cam_intrinsics, robot.cam_pose, workspace_limits, heightmap_resolution)
valid_depth_heightmap = depth_heightmap.copy()
valid_depth_heightmap[np.isnan(valid_depth_heightmap)] = 0

robot.graph_move_to(position_fl,tool_orientation,valid_depth_heightmap)
