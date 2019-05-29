#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import time
import cv2
from real.camera import Camera
from robot import Robot
    

# User options (change me)
# --------------- Setup options ---------------
tcp_host_ip = '192.168.1.5' # IP and port to robot arm as TCP client (UR5)
tcp_port = 30002
rtc_host_ip = '100.127.7.223' # IP and port to robot arm as real-time client (UR5)
rtc_port = 30003
workspace_limits = np.asarray([[-0.27, 0.24], [0.14,0.4], [0.025, 0.15]]) # Cols: min max, Rows: u v z (define workspace limits in alternative coordinates)
#tool_orientation = [1.2309841027750899, -1.6417700286074282, -2.0187284668064036]
tool_orientation = [-2.8423740026858066, -1.2878981887531054, -0.047896657364744154]
# ---------------------------------------------


# Move robot to home pose
robot = Robot(False, None, None, workspace_limits,
              tcp_host_ip, tcp_port, rtc_host_ip, rtc_port,
              False, None, None)
SECURITY_POSITION = [0.18175647976737064, 0.22399351224588684, 0.22741012259139293]
#SECURITY_ORIENTATION = [-2.342346400947404, 0.49086167950043563, -0.015532489811231705]
#robot.move_to(SECURITY_POSITION,SECURITY_ORIENTATION)
#robot.open_gripper()

# Slow down robot
robot.joint_acc = 1.4
robot.joint_vel = 1.05

# Callback function for clicking on OpenCV window
click_point_pix = ()
camera_color_img, camera_depth_img = robot.get_camera_data()
def mouseclick_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        global camera, robot, click_point_pix
        click_point_pix = (x,y)

        # Get click point in camera coordinates
        click_z = camera_depth_img[y][x] * robot.cam_depth_scale
        click_x = np.multiply(x-robot.cam_intrinsics[0][2],click_z/robot.cam_intrinsics[0][0])
        click_y = np.multiply(y-robot.cam_intrinsics[1][2],click_z/robot.cam_intrinsics[1][1])
        if click_z == 0:
            return
        click_point = np.asarray([click_x,click_y,click_z])
        click_point.shape = (3,1)

        # Convert camera to robot coordinates
        # camera2robot = np.linalg.inv(robot.cam_pose)
        camera2robot = robot.cam_pose
        target_position = np.dot(camera2robot[0:3,0:3],click_point) + camera2robot[0:3,3:]

        target_position = target_position[0:3,0]
        print(target_position)
        robot.push(target_position,np.pi/4,workspace_limits)


# Show color and depth frames
cv2.namedWindow('color', cv2.WINDOW_NORMAL)
cv2.resizeWindow('color', 1800, 900)
cv2.setMouseCallback('color', mouseclick_callback)
cv2.namedWindow('depth')

while True:
    camera_color_img, camera_depth_img = robot.get_camera_data()
    bgr_data = cv2.cvtColor(camera_color_img, cv2.COLOR_RGB2BGR)
    if len(click_point_pix) != 0:
        bgr_data = cv2.circle(bgr_data, click_point_pix, 7, (0,0,255), 2)
    cv2.imshow('color', bgr_data)
    cv2.imshow('depth', camera_depth_img)
    
    if cv2.waitKey(1) == ord('c'):
        break

cv2.destroyAllWindows()
