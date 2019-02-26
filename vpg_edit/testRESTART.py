#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import time
import cv2
import utilsedit
from real.camera import Camera
from robot import Robot


# User options (change me)
# --------------- Setup options ---------------
tcp_host_ip = '192.168.1.5' # IP and port to robot arm as TCP client (UR3)
tcp_port = 30002
rtc_host_ip = '100.127.7.223' # IP and port to robot arm as real-time client (UR3)
rtc_port = 30003
workspace_limits = np.asarray([[-0.27, 0.24], [0.13,0.4], [0.005, 0.15]]) # Cols: min max, Rows: u v z (define workspace limits in alternative coordinates)
home_position = [0.15521074915228592, 0.18536901992860863, 0.10333937281420481]
heightmap_resolution = 0.0025 
# ---------------------------------------------

# Initializing robot
robot = Robot(False, None, None, workspace_limits,
              tcp_host_ip, tcp_port, rtc_host_ip, rtc_port,
              False, None, None)
robot.joint_acc = 1.4
robot.joint_vel = 1.05

robot.restart_real()

## Get latest RGB-D image
#color_img, depth_img = robot.get_camera_data()
#depth_img = depth_img * robot.cam_depth_scale # Apply depth scale from calibration
#
## Get heightmap from RGB-D image (by re-projecting 3D point cloud)
#color_heightmap, depth_heightmap = utilsedit.get_heightmap(color_img, depth_img, robot.cam_intrinsics, robot.cam_pose, workspace_limits, heightmap_resolution)
#valid_depth_heightmap = depth_heightmap.copy()
#valid_depth_heightmap[np.isnan(valid_depth_heightmap)] = 0
## Reset simulation or pause real-world training if table is empty
#stuff_count = np.zeros(valid_depth_heightmap.shape)
#stuff_count[valid_depth_heightmap > 0.02] = 1
#empty_threshold = 300
##if is_sim and is_testing:
##    empty_threshold = 10  
#if np.sum(stuff_count) < empty_threshold: #or (is_sim and no_change_count[0] + no_change_count[1] > 10):
#    no_change_count = [0, 0]
##    if is_sim:
##        print('Not enough objects in view (value: %d)! Repositioning objects.' % (np.sum(stuff_count)))
##        robot.restart_sim()
##        robot.add_objects()
##        if is_testing: # If at end of test run, re-load original weights (before test run)
##            trainer.model.load_state_dict(torch.load(snapshot_file))
##    else:
#        # print('Not enough stuff on the table (value: %d)! Pausing for 30 seconds.' % (np.sum(stuff_count)))
#        # time.sleep(30)
#    print('Not enough stuff on the table (value: %d)! Flipping over bin of objects...' % (np.sum(stuff_count)))
#    robot.restart_real()
