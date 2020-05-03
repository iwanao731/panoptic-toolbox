import json
import os
import array
import panutils
import numpy as np
import cv2
import open3d
from scipy.spatial.transform import Rotation

def projectPoints(X, K, camera_pose, Kd):

    R, t = decomposeRt(camera_pose)
    x = np.array((R.dot(X)).transpose() + t)
    x = x.transpose()

    x[0:2,:] = x[0:2,:]/x[2,:]
    r = x[0,:]*x[0,:] + x[1,:]*x[1,:]

    x[0,:] = x[0,:]*(1 + Kd[0]*r + Kd[1]*r*r + Kd[4]*r*r*r) + 2*Kd[2]*x[0,:]*x[1,:] + Kd[3]*(r*r + 2*x[0,:]*x[0,:])
    x[1,:] = x[1,:]*(1 + Kd[0]*r + Kd[1]*r*r + Kd[4]*r*r*r) + 2*Kd[3]*x[0,:]*x[1,:] + Kd[2]*(r*r + 2*x[1,:]*x[1,:])

    x[0,:] = K[0,0]*x[0,:] + K[0,1]*x[1,:] + K[0,2]
    x[1,:] = K[1,0]*x[0,:] + K[1,1]*x[1,:] + K[1,2]

    return x

def decomposeRt(mat4):
    R = np.zeros((3,3))    
    t = np.zeros(3)
    R[0][0] = mat4[0][0]
    R[0][1] = mat4[0][1]
    R[0][2] = mat4[0][2]
    R[1][0] = mat4[1][0]
    R[1][1] = mat4[1][1]
    R[1][2] = mat4[1][2]
    R[2][0] = mat4[2][0]
    R[2][1] = mat4[2][1]
    R[2][2] = mat4[2][2]
    t[0] = mat4[0][3]
    t[1] = mat4[1][3]
    t[2] = mat4[2][3]
    return R, t
    
def composeMat4(R, t):
    mat4 = np.zeros((4,4))
    mat4[0][0] = R[0,0]
    mat4[0][1] = R[0,1]
    mat4[0][2] = R[0,2]
    mat4[1][0] = R[1,0]
    mat4[1][1] = R[1,1]
    mat4[1][2] = R[1,2]
    mat4[2][0] = R[2,0]
    mat4[2][1] = R[2,1]
    mat4[2][2] = R[2,2]
    mat4[0][3] = t[0]
    mat4[1][3] = t[1]
    mat4[2][3] = t[2]
    mat4[3][3] = 1.0
    return mat4

def readDepthIndex_1basedIdx(fname, idx):
    number_of_pixels = 512*424
    #print(fname)
    file = open(fname, 'rb')
    step = 2 * number_of_pixels # because 2 of bite
    file.seek(step*(idx-1))
    image_value = array.array('h')  # 'h': signed short (2 byte = 16 bit)
    image_value.fromfile(file, number_of_pixels)
    image = (np.asanyarray(image_value, dtype=np.uint16)).reshape((424, 512))
    image = cv2.flip(image, 1)
    file.close()
    return image

def makevolume(rgb, depth, camera_pose, camCalibData, volume):
    k_dist = np.matrix(camCalibData['K_depth'])
    m_dist = np.matrix(camCalibData['M_depth'])[0:3,:]
    distCoeffs = np.array(camCalibData['distCoeffs_depth'])
    width = 512
    height = 424
    cx = k_dist[0,2]
    cy = k_dist[1,2]
    fx = k_dist[0,0]
    fy = k_dist[1,1]
    intrinsic = open3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)
    depth =  open3d.geometry.Image(depth)

    pcd = open3d.geometry.create_point_cloud_from_depth_image(depth, intrinsic)

    # pcd * M_color
    # 
    #color_camera_pose
    #depth_camera_pose
    projected_points_onto_rgb_image_plane = projectPoints(np.matrix(pcd.points).transpose(), np.matrix(camCalibData['K_color']), camera_pose, np.array(camCalibData['distCoeffs_color']))
    projected_points_onto_depth_image_plane = projectPoints(np.matrix(pcd.points).transpose(), np.matrix(camCalibData['K_depth']), camera_pose, np.array(camCalibData['distCoeffs_depth']))
    rgb_fit_depth = np.zeros((height,width,3), np.uint8)

    for i in range(len(pcd.points)):
        u_d = int(projected_points_onto_depth_image_plane[0,i])
        v_d = int(projected_points_onto_depth_image_plane[1,i])
        u_c = int(projected_points_onto_rgb_image_plane[0,i])
        v_c = int(projected_points_onto_rgb_image_plane[1,i])

        rgb_fit_depth[v_d,u_d] = rgb[v_c,u_c]

    rgb_resize = cv2.resize(rgb, (512, 424))
    for u in range(width):
        for v in range(height):
            rgb_fit_depth[v,u] = rgb_resize[v, u]

    # need to make color data to be as same resolution as depth
    rgb_fit_depth = cv2.cvtColor(rgb_fit_depth, cv2.COLOR_BGR2RGB)

    color = open3d.geometry.Image(np.asanyarray(rgb_fit_depth, dtype=np.uint8))
    rgbd =  open3d.geometry.create_rgbd_image_from_color_and_depth(color, depth, convert_rgb_to_intensity = False)
    volume.integrate(rgbd, intrinsic, np.linalg.inv(camera_pose))

