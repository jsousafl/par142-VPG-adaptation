#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import time
import cv2
import utilsedit
from real.camera import Camera
from robot import Robot
#from logger import Logger

    

# User options (change me)
# --------------- Setup options ---------------
tcp_host_ip = '192.168.1.5' # IP and port to robot arm as TCP client (UR3)
tcp_port = 30002
rtc_host_ip = '100.127.7.223' # IP and port to robot arm as real-time client (UR3)
rtc_port = 30003
workspace_limits = np.asarray([[-0.112, 0.112], [0.16, 0.384], [0.0, 0.12]]) # Cols: min max, Rows: u v z (define workspace limits in alternative coordinates)
home_position = [0.15521074915228592, 0.18536901992860863, 0.10333937281420481]
tool_orientation = [-2.8423740026858066, -1.2878981887531054, -0.047896657364744154]
heightmap_resolution = 0.00175
# ---------------------------------------------

# Initializing robot
robot = Robot(False, None, None, workspace_limits,
              tcp_host_ip, tcp_port, rtc_host_ip, rtc_port,
              False, None, None)
robot.joint_acc = 1.4
robot.joint_vel = 1.05
#logger = Logger(False, 'logs')
#robot.move_to(home_position,tool_orientation)

#camera_color_img, camera_depth_img = robot.get_camera_data()
#heightmap_resolution = 0.0025 #defqult value took of the main.py
#surface_pts, color_pts = utilsedit.get_pointcloud(camera_color_img, camera_depth_img, robot.cam_intrinsics)
#color_heigthmap, depth_heigthmap = utilsedit.get_heightmap(camera_color_img,camera_depth_img,robot.cam_intrinsics, robot.cam_pose, workspace_limits, heightmap_resolution)
#plt.subplot(211)
#axes = plt.gca()
##axes.set_xlim([0,150])
#axes.set_ylim([0,110])
#plt.imshow(color_heigthmap)
#plt.subplot(212)
#plt.imshow(depth_heigthmap)
#plt.show()


# Callback function for clicking on OpenCV window
click_point_pix = ()
camera_color_img, camera_depth_img = robot.get_camera_data()
def mouseclick_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        global camera, robot, click_point_pix
        click_point_pix = (x,y)
        print(click_point_pix)
        color_heigthmap, depth_heigthmap = utilsedit.get_heightmap(camera_color_img,camera_depth_img,robot.cam_intrinsics, robot.cam_pose, workspace_limits, heightmap_resolution)
#        # Get click point in camera coordinates
#        click_z = camera_depth_img[y][x] * robot.cam_depth_scale
#        click_x = np.multiply(x-robot.cam_intrinsics[0][2],click_z/robot.cam_intrinsics[0][0])
#        click_y = np.multiply(y-robot.cam_intrinsics[1][2],click_z/robot.cam_intrinsics[1][1])
#        if click_z == 0:
#            return
#        click_point = np.asarray([click_x,click_y,click_z])
#        click_point.shape = (3,1)
#
#        # Convert camera to robot coordinates
#        # camera2robot = np.linalg.inv(robot.cam_pose)
        
        valid_depth_heightmap = depth_heigthmap.copy()
        valid_depth_heightmap[np.isnan(valid_depth_heightmap)] = 0        
        target_u = workspace_limits[0][1] - (x-48)*heightmap_resolution
        target_v = workspace_limits[1][0] + (y-30)*heightmap_resolution
        target_z = min(max(valid_depth_heightmap[y][x],workspace_limits[2][0]),workspace_limits[2][1])
        position_u_v = np.transpose(np.array([target_u,target_v,target_z]))
#        print(position_u_v)
        target_position = utilsedit.altern2cart(position_u_v)
#        camera2robot = robot.cam_pose
#        target_position = np.dot(camera2robot[0:3,0:3],click_point) + camera2robot[0:3,3:]
#
#        target_position = target_position[0:3,0]
        print(target_position)
        robot.push(target_position,-np.pi/2,workspace_limits)

cv2.namedWindow('color', cv2.WINDOW_NORMAL)
cv2.resizeWindow('color', 1800, 900)
cv2.setMouseCallback('color', mouseclick_callback)

while True:
    camera_color_img, camera_depth_img = robot.get_camera_data()
    surface_pts, color_pts = utilsedit.get_pointcloud(camera_color_img, camera_depth_img, robot.cam_intrinsics)
    color_heigthmap, depth_heigthmap = utilsedit.get_heightmap(camera_color_img,camera_depth_img,robot.cam_intrinsics, robot.cam_pose, workspace_limits, heightmap_resolution)
    bgr_data = cv2.cvtColor(color_heigthmap, cv2.COLOR_RGB2BGR)
    valid_depth_heightmap = depth_heigthmap.copy()
    valid_depth_heightmap[np.isnan(valid_depth_heightmap)] = 0  
#    logger.save_heightmaps(1, color_heigthmap, valid_depth_heightmap, '0')
    #bgr_data_flip = cv2.flip(bgr_data,0)
    bgr_data_flip = bgr_data
    if len(click_point_pix) != 0:
        bgr_data_flip = cv2.circle(bgr_data_flip, click_point_pix, 3, (0,0,255), 2)
    cv2.imshow('color', bgr_data_flip)
    
    if cv2.waitKey(1) == ord('c'):
        break
cv2.destroyAllWindows()


