#!/usr/bin/env python
import math
import matplotlib.pyplot as plt
import numpy as np
import time
import cv2
from real.camera import Camera
from robot import Robot
from scipy import optimize  
from mpl_toolkits.mplot3d import Axes3D  


#
tcp_host_ip = '192.168.1.5' # IP and port to robot arm as TCP client (UR5)
tcp_port = 30002
rtc_host_ip = '100.127.7.223' # IP and port to robot arm as real-time client (UR5)
rtc_port = 30003
workspace_limits = np.asarray([[-0.105, 0.432], [-0.054, 0.468], [0.0, 0.045]]) # Cols: min max, Rows: x y z (define workspace limits in robot coordinates)
checkerboard_offset_from_tool = [-0.0478,0.0378,-0.04]
tool_orientation = [-2.8423740026858066, -1.2878981887531054, -0.047896657364744154] #DEFINED AND MEASURED ORIENTATION
point_origin_cube = [-0.065,0.262, 0.0] # DEFINED AND MESURED ORIGIN POINT (FACE 1 AND 2)
point_end_face2 = [0.269,-0.054,0.0] # DEFINED AND MESURED END POINT OF FACE 2
point_end_face1 = [0.096,0.449,0.0] # DEFINED AND MESURED END POINT OF FACE 1

# DEFINITION OF CALIBRATION GRID 
#######################################################################################################33
calib_grid_step = 0.0225
def intercalate(L,num_divisions):
    num_elements = int(len(L)/num_divisions)
    intercalate = []
    for j in range(num_elements):
        for i in range(num_divisions):
            intercalate.append(L[j + i*num_elements])
    return intercalate
    

num_step_face1 = 3
step_face1 = (point_end_face1[0]-point_origin_cube[0])/num_step_face1
   
gridspace_z = np.linspace(workspace_limits[2][0], workspace_limits[2][1], 1 + (workspace_limits[2][1] - workspace_limits[2][0])/calib_grid_step)

a_perpendicular = math.tan(138.2/180*math.pi) # angle entre la face 1 et l'axe x
b_perpendicular = point_origin_cube[1]-a_perpendicular*point_origin_cube[0]
num_step_face2 = 3
step_face2 = (point_end_face2[0]-point_origin_cube[0])/num_step_face2

a = math.tan((138.2-90)/180*math.pi) # angle entre la face 2 et l'axe x

x_initial = np.zeros(num_step_face2+1)
y_initial = np.zeros(num_step_face2+1)
x = np.array([])
y = np.array([])

for i in range(num_step_face2+1):
    x_initial[i] = point_origin_cube[0] + i*step_face2
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
calib_grid_z.shape = (num_calib_grid_pts,1)
calib_grid_x = np.array(calib_grid_x)
calib_grid_x = intercalate(calib_grid_x,len(gridspace_z))
calib_grid_y = intercalate(calib_grid_y,len(gridspace_z))
calib_grid_pts = np.concatenate((calib_grid_x, calib_grid_y, z), axis=1)
##########################################################################################################
#ROBOT SETUP
robot = Robot(False, None, None, workspace_limits,
              tcp_host_ip, tcp_port, rtc_host_ip, rtc_port,
              False, None, None)

SECURITY_POSITION = [0.1642542441737147, 0.22407768353652058, 0.09311416877891272]
SECURITY_ORIENTATION = [-2.8095352019503803, -1.143738023156609, -0.10212882587827629]

robot.move_to(SECURITY_POSITION,SECURITY_ORIENTATION)
time.sleep(0.5)

robot.joint_acc = 1
robot.joint_vel = 1
#############################################################################################################
#START OF CALIBRATION

measured_pts = []
observed_pts = []
observed_pix = []

# Move robot to each calibration point in workspace
print('Collecting data...')
for calib_pt_idx in range(num_calib_grid_pts):
    tool_position = calib_grid_pts[calib_pt_idx,:]
    robot.move_to(tool_position, tool_orientation)
    time.sleep(0.5)
    
    
    # Find checkerboard center
    checkerboard_size = (3,3)
    refine_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    camera_color_img, camera_depth_img = robot.get_camera_data()
    bgr_color_data = cv2.cvtColor(camera_color_img, cv2.COLOR_RGB2BGR)
    gray_data = cv2.cvtColor(bgr_color_data, cv2.COLOR_RGB2GRAY)
    checkerboard_found, corners = cv2.findChessboardCorners(gray_data, checkerboard_size, None, cv2.CALIB_CB_ADAPTIVE_THRESH)
    if checkerboard_found:
        corners_refined = cv2.cornerSubPix(gray_data, corners, (3,3), (-1,-1), refine_criteria)
        
        print('--------------------------DETECTION---------------------------------')
        
        # Get observed checkerboard center 3D point in camera space
        checkerboard_pix = np.round(corners_refined[4,0,:]).astype(int)
        checkerboard_z = camera_depth_img[checkerboard_pix[1]][checkerboard_pix[0]]
        checkerboard_x = np.multiply(checkerboard_pix[0]-robot.cam_intrinsics[0][2],checkerboard_z/robot.cam_intrinsics[0][0])
        checkerboard_y = np.multiply(checkerboard_pix[1]-robot.cam_intrinsics[1][2],checkerboard_z/robot.cam_intrinsics[1][1])
        if checkerboard_z == 0:
            continue

        # Save calibration point and observed checkerboard center
        observed_pts.append([checkerboard_x,checkerboard_y,checkerboard_z])
        # tool_position[2] += checkerboard_offset_from_tool
        tool_position = tool_position - checkerboard_offset_from_tool

        measured_pts.append(tool_position)
        observed_pix.append(checkerboard_pix)

        # Draw and display the corners
        # vis = cv2.drawChessboardCorners(robot.camera.color_data, checkerboard_size, corners_refined, checkerboard_found)
        vis = cv2.drawChessboardCorners(bgr_color_data, (1,1), corners_refined[4,:,:], checkerboard_found)
        cv2.imwrite('%06d.png' % len(measured_pts), vis)
        #cv2.imshow('Calibration',vis)
        cv2.waitKey(10)

# Move robot back to home pose
#robot.go_home()

measured_pts = np.asarray(measured_pts)
observed_pts = np.asarray(observed_pts)
observed_pix = np.asarray(observed_pix)
world2camera = np.eye(4)

# Estimate rigid transform with SVD (from Nghia Ho)
def get_rigid_transform(A, B):
    assert len(A) == len(B)
    N = A.shape[0]; # Total points
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - np.tile(centroid_A, (N, 1)) # Centre the points
    BB = B - np.tile(centroid_B, (N, 1))
    H = np.dot(np.transpose(AA), BB) # Dot is matrix multiplication for array
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)
    if np.linalg.det(R) < 0: # Special reflection case
       Vt[2,:] *= -1
       R = np.dot(Vt.T, U.T)
    t = np.dot(-R, centroid_A.T) + centroid_B.T
    return R, t

def get_rigid_transform_error(z_scale):
    global measured_pts, observed_pts, observed_pix, world2camera, camera

    # Apply z offset and compute new observed points using camera intrinsics
    observed_z = observed_pts[:,2:] * z_scale
    observed_x = np.multiply(observed_pix[:,[0]]-robot.cam_intrinsics[0][2],observed_z/robot.cam_intrinsics[0][0])
    observed_y = np.multiply(observed_pix[:,[1]]-robot.cam_intrinsics[1][2],observed_z/robot.cam_intrinsics[1][1])
    new_observed_pts = np.concatenate((observed_x, observed_y, observed_z), axis=1)

    # Estimate rigid transform between measured points and new observed points
    R, t = get_rigid_transform(np.asarray(measured_pts), np.asarray(new_observed_pts))
    t.shape = (3,1)
    world2camera = np.concatenate((np.concatenate((R, t), axis=1),np.array([[0, 0, 0, 1]])), axis=0)

    # Compute rigid transform error
    registered_pts = np.dot(R,np.transpose(measured_pts)) + np.tile(t,(1,measured_pts.shape[0]))
    error = np.transpose(registered_pts) - new_observed_pts
    error = np.sum(np.multiply(error,error))
    rmse = np.sqrt(error/measured_pts.shape[0]);
    return rmse

# Optimize z scale w.r.t. rigid transform error
print('Calibrating...')
z_scale_init = 1
optim_result = optimize.minimize(get_rigid_transform_error, np.asarray(z_scale_init), method='Nelder-Mead')
camera_depth_offset = optim_result.x

# Save camera optimized offset and camera pose
print('Saving...')
np.savetxt('real/camera_depth_scale.txt', camera_depth_offset, delimiter=' ')
get_rigid_transform_error(camera_depth_offset)
camera_pose = np.linalg.inv(world2camera)
np.savetxt('real/camera_pose.txt', camera_pose, delimiter=' ')
print('Number of points')
print(len(measured_pts))
print('Number of observed points')
print(len(observed_pts))

print('Done.')

# DEBUG CODE -----------------------------------------------------------------------------------

# np.savetxt('measured_pts.txt', np.asarray(measured_pts), delimiter=' ')
# np.savetxt('observed_pts.txt', np.asarray(observed_pts), delimiter=' ')
# np.savetxt('observed_pix.txt', np.asarray(observed_pix), delimiter=' ')
# measured_pts = np.loadtxt('measured_pts.txt', delimiter=' ')
# observed_pts = np.loadtxt('observed_pts.txt', delimiter=' ')
# observed_pix = np.loadtxt('observed_pix.txt', delimiter=' ')

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.scatter(measured_pts[:,0],measured_pts[:,1],measured_pts[:,2], c='blue')

# print(camera_depth_offset)
# R, t = get_rigid_transform(np.asarray(measured_pts), np.asarray(observed_pts))
# t.shape = (3,1)
# camera_pose = np.concatenate((np.concatenate((R, t), axis=1),np.array([[0, 0, 0, 1]])), axis=0)
# camera2robot = np.linalg.inv(camera_pose)
# t_observed_pts = np.transpose(np.dot(camera2robot[0:3,0:3],np.transpose(observed_pts)) + np.tile(camera2robot[0:3,3:],(1,observed_pts.shape[0])))

# ax.scatter(t_observed_pts[:,0],t_observed_pts[:,1],t_observed_pts[:,2], c='red')

# new_observed_pts = observed_pts.copy()
# new_observed_pts[:,2] = new_observed_pts[:,2] * camera_depth_offset[0]
# R, t = get_rigid_transform(np.asarray(measured_pts), np.asarray(new_observed_pts))
# t.shape = (3,1)
# camera_pose = np.concatenate((np.concatenate((R, t), axis=1),np.array([[0, 0, 0, 1]])), axis=0)
# camera2robot = np.linalg.inv(camera_pose)
# t_new_observed_pts = np.transpose(np.dot(camera2robot[0:3,0:3],np.transpose(new_observed_pts)) + np.tile(camera2robot[0:3,3:],(1,new_observed_pts.shape[0])))

# ax.scatter(t_new_observed_pts[:,0],t_new_observed_pts[:,1],t_new_observed_pts[:,2], c='green')

# plt.show()