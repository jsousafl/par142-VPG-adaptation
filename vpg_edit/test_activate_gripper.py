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
tool_orientation = [-2.8423740026858066, -1.2878981887531054, -0.047896657364744154]
heightmap_resolution = 0.0025 
# ---------------------------------------------

# Initializing robot
robot = Robot(False, None, None, workspace_limits,
              tcp_host_ip, tcp_port, rtc_host_ip, rtc_port,
              False, None, None)
robot.joint_acc = 1.4
robot.joint_vel = 1.05

robot.activate_gripper()

