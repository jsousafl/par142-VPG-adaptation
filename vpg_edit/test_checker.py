#!/usr/bin/env python 3
import numpy as np
import cv2
from real.camera import Camera

tcp_host_ip = '192.168.1.5' # IP and port to robot arm as TCP client (UR5)
tcp_port = 30002
rtc_host_ip = '100.127.7.223' # IP and port to robot arm as real-time client (UR5)
rtc_port = 30003
#workspace_limits = np.asarray([[-0.112, 0.0], [0.35, 0.49], [0.05, 0.15]]) # Cols: min max, Rows: x y z (define workspace limits in robot coordinates)
workspace_limits = np.asarray([[-0.105, 0.432], [-0.054, 0.468], [0.0, 0.045]]) # Cols: min max, Rows: x y z (define workspace limits in robot coordinates)
checkerboard_offset_from_tool = [-0.0478,0.0378,-0.04]
tool_position = [workspace_limits[0][0],0.3168935073624113, 0.044575957549417684] #[0.05,0.35,0.05]
calib_grid_step = 0.05

measured_pts = []
observed_pts = []
observed_pix = []

camera = Camera()
print('Collecting data...')

# Find checkerboard center
checkerboard_size = (3,3) # Use (3,3) to 4x4 checkerboard
refine_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
camera_color_img, camera_depth_img = camera.get_data()
bgr_color_data = cv2.cvtColor(camera_color_img, cv2.COLOR_RGB2BGR)
gray_data = cv2.cvtColor(bgr_color_data, cv2.COLOR_RGB2GRAY)
checkerboard_found, corners = cv2.findChessboardCorners(gray_data, checkerboard_size, None, cv2.CALIB_CB_ADAPTIVE_THRESH)
if checkerboard_found:
    print("Checkerboard detected")
    corners_refined = cv2.cornerSubPix(gray_data, corners, (3,3), (-1,-1), refine_criteria)

    # Get observed checkerboard center 3D point in camera space
    checkerboard_pix = np.round(corners_refined[4,0,:]).astype(int)
    checkerboard_z = camera_depth_img[checkerboard_pix[1]][checkerboard_pix[0]]
    checkerboard_x = np.multiply(checkerboard_pix[0]-camera.intrinsics[0][2],checkerboard_z/camera.intrinsics[0][0])
    checkerboard_y = np.multiply(checkerboard_pix[1]-camera.intrinsics[1][2],checkerboard_z/camera.intrinsics[1][1])

    # Save calibration point and observed checkerboard center
    observed_pts.append([checkerboard_x,checkerboard_y,checkerboard_z])

    tool_position = tool_position + checkerboard_offset_from_tool

    measured_pts.append(tool_position)
    observed_pix.append(checkerboard_pix)

    # Draw and display the corners
    vis = cv2.drawChessboardCorners(bgr_color_data, (1,1), corners_refined[4,:,:], checkerboard_found)
    cv2.imwrite('%checkerboard_detected.png' % len(measured_pts), vis)
    cv2.imshow('Calibration',vis)
    cv2.waitKey(10)