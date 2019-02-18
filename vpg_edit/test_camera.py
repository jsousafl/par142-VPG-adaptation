# -*- coding: utf-8 -*-
"""
Created on Thu Nov 15 13:46:52 2018

@author: liris-user
"""

import socket
import numpy as np
import cv2
import os
import time
import struct
#import matplotlib.pyplot as plt
import math
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.autograd import Variable

def capture():
    while True:
        color_img, depth_img = camera.get_data()
        plt.subplot(211)
        plt.imshow(color_img)
        plt.subplot(212)
        plt.imshow(depth_img)
        plt.show()


def get_pointcloud(color_img, depth_img, camera_intrinsics):

    # Get depth image size
    im_h = depth_img.shape[0]
    im_w = depth_img.shape[1]

    # Project depth into 3D point cloud in camera coordinates
    pix_x,pix_y = np.meshgrid(np.linspace(0,im_w-1,im_w), np.linspace(0,im_h-1,im_h))
    cam_pts_x = np.multiply(pix_x-camera_intrinsics[0][2],depth_img/camera_intrinsics[0][0])
    cam_pts_y = np.multiply(pix_y-camera_intrinsics[1][2],depth_img/camera_intrinsics[1][1])
    cam_pts_z = depth_img.copy()
    cam_pts_x.shape = (im_h*im_w,1)
    cam_pts_y.shape = (im_h*im_w,1)
    cam_pts_z.shape = (im_h*im_w,1)

    # Reshape image into colors for 3D point cloud
    rgb_pts_r = color_img[:,:,0]
    rgb_pts_g = color_img[:,:,1]
    rgb_pts_b = color_img[:,:,2]
    rgb_pts_r.shape = (im_h*im_w,1)
    rgb_pts_g.shape = (im_h*im_w,1)
    rgb_pts_b.shape = (im_h*im_w,1)

    cam_pts = np.concatenate((cam_pts_x, cam_pts_y, cam_pts_z), axis=1)
    rgb_pts = np.concatenate((rgb_pts_r, rgb_pts_g, rgb_pts_b), axis=1)

    return cam_pts, rgb_pts

# Fetch RGB-D data from RealSense camera
from real.camera import Camera
print('before cam')
camera = Camera()
print('after cam')
time.sleep(1) # Give camera some time to load data
#color_img, depth_img = camera.get_data()
#cam_intrinsics = np.asarray([[618.62, 0, 320], [0, 618.62, 240], [0, 0, 1]])
#
#cam_pts, rgb_pts = get_pointcloud(color_img, depth_img, cam_intrinsics)
#print(cam_pts)
#print(rgb_pts)

# Find checkerboard center
measured_pts = []
observed_pts = []
observed_pix = []

print('gjdlgwlwdwdw')
cam_intrinsics = np.asarray([[618.62, 0, 320], [0, 618.62, 240], [0, 0, 1]])
checkerboard_size = (3,3)
refine_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
camera_color_img, camera_depth_img = camera.get_data()
bgr_color_data = cv2.cvtColor(camera_color_img, cv2.COLOR_RGB2BGR)
gray_data = cv2.cvtColor(bgr_color_data, cv2.COLOR_RGB2GRAY)
checkerboard_found, corners = cv2.findChessboardCorners(gray_data, checkerboard_size, None, cv2.CALIB_CB_ADAPTIVE_THRESH)
print(corners)
#plt.imshow(gray_data)
#plt.show()
if checkerboard_found:
    corners_refined = cv2.cornerSubPix(gray_data, corners, (3,3), (-1,-1), refine_criteria) # Refines the corner locations.

    # Get observed checkerboard center 3D point in camera space
    checkerboard_pix = np.round(corners_refined[4,0,:]).astype(int)
    checkerboard_z = camera_depth_img[checkerboard_pix[1]][checkerboard_pix[0]]
    checkerboard_x = np.multiply(checkerboard_pix[0]-cam_intrinsics[0][2],checkerboard_z/cam_intrinsics[0][0])
    checkerboard_y = np.multiply(checkerboard_pix[1]-cam_intrinsics[1][2],checkerboard_z/cam_intrinsics[1][1])
    if checkerboard_z == 0:
        print('test')

    # Save calibration point and observed checkerboard center
    observed_pts.append([checkerboard_x,checkerboard_y,checkerboard_z])

    observed_pix.append(checkerboard_pix)

    # Draw and display the corners
    # vis = cv2.drawChessboardCorners(robot.camera.color_data, checkerboard_size, corners_refined, checkerboard_found)
    vis = cv2.drawChessboardCorners(bgr_color_data, (1,1), corners_refined[4,:,:], checkerboard_found)
    cv2.imwrite('%06d.png' % len(measured_pts), vis)
    cv2.imshow('Calibration',vis)
    cv2.waitKey(1000)