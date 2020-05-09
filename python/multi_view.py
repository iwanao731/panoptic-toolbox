import cv2
import os
import json
import numpy as np
import open3d as o3d
import sys
sys.path.append(os.path.abspath('../'))
import pyrgbd
from open3d.open3d.geometry import create_rgbd_image_from_color_and_depth
from pyrgbd.util import *

def parse_camera_params(kinect):
    param = {}
    param['depth'] = {}
    param['depth']['fx'] = kinect['K_depth'][0][0]
    param['depth']['fy'] = kinect['K_depth'][1][1]
    param['depth']['cx'] = kinect['K_depth'][0][2]
    param['depth']['cy'] = kinect['K_depth'][1][2]
    param['depth']['K'] = np.array(kinect['K_depth'])
    # ignore distCoeffs_depth's 5th (1000) and 6th (0) element
    # since they are strange
    param['depth']['distCoeffs'] = np.array(kinect['distCoeffs_depth'][:5])

    param['color'] = {}
    param['color']['fx'] = kinect['K_color'][0][0]
    param['color']['fy'] = kinect['K_color'][1][1]
    param['color']['cx'] = kinect['K_color'][0][2]
    param['color']['cy'] = kinect['K_color'][1][2]
    param['color']['K'] = np.array(kinect['K_color'])
    # ignore distCoeffs_color's 5th (1000) and 6th (0) element
    # since they are strange
    param['color']['distCoeffs'] = np.array(kinect['distCoeffs_color'][:5])

    d_T = np.array(kinect['M_depth'])
    c_T = np.array(kinect['M_color'])
    # "d_T @" is important...
    c2d_T = d_T @ c_T
    d2c_T = np.linalg.inv(c2d_T)

    param['d2c_R'] = d2c_T[0:3, 0:3]
    param['d2c_t'] = d2c_T[0:3, 3]

    # world to depth
    # "d_T @" is important...
    w2d_T = d_T @  np.array(kinect['M_world2sensor'])
    d2w_T = np.linalg.inv(w2d_T)
    param['w2d_R'] = w2d_T[0:3, 0:3]
    param['w2d_t'] = w2d_T[0:3, 3]
    param['w2d_T'] = w2d_T

    param['d2w_R'] = d2w_T[0:3, 0:3]
    param['d2w_t'] = d2w_T[0:3, 3]
    param['d2w_T'] = d2w_T

    return param


if __name__ == '__main__':
    data_dir = '../171026_cello3/'
    kinect_params = load_json(os.path.join(data_dir,
                                           'kcalibration_171026_cello3.json'))

    KINECT_NUM = 10
    start_frame = 400
    last_frame = 401
    background_removal_threshold = 3.2
    frame_id_list = [i for i in range(start_frame, last_frame)]
    cameras = loadCalibrationData(os.path.join(data_dir,
                                           'calibration_171026_cello3.json'))

    for frame_id in frame_id_list:

        global_pc = []
        global_pc_color = []

        volume = o3d.integration.ScalableTSDFVolume(
            voxel_length=5.0 / 512.0,
            sdf_trunc=0.05,
            color_type=o3d.integration.TSDFVolumeColorType.RGB8)

        for i in range(KINECT_NUM):
            param = parse_camera_params(kinect_params['sensors'][i])
            dfx, dfy, dcx, dcy = param['depth']['fx'], param['depth']['fy'], \
                param['depth']['cx'], param['depth']['cy']
            cfx, cfy, ccx, ccy = param['color']['fx'], param['color']['fy'], \
                param['color']['cx'], param['color']['cy']
            ddistpr = param['depth']['distCoeffs']
            cdistpr = param['color']['distCoeffs']            

            color = cv2.imread(os.path.join(
                data_dir, 'kinectSync/color/{0}/{0}_{1}.jpg'.format(cameras[i]['name'], str('{:0=8}'.format(frame_id)))))

            depth = cv2.imread(os.path.join(
                data_dir, 'kinectSync/depth/{0}/{0}_{1}.png'.format(cameras[i]['name'], str('{:0=8}'.format(frame_id)))), -1)

            depth = depth.astype(np.float) / 1000.0  # convert to meter scale
            depth = depth.astype(np.float32)  # to float32
            # Median filter to remove noise
            depth = medianBlurForDepthWithNoHoleFilling(depth, 3)
            mapped_color, valid_mask = pyrgbd.gen_mapped_color(depth, dfx, dfy,
                                                               dcx, dcy,
                                                               color, cfx, cfy,
                                                               ccx, ccy,
                                                               param['d2c_R'],
                                                               param['d2c_t'],
                                                               ddist_type='OPENCV',
                                                               ddist_param=ddistpr,
                                                               cdist_type='OPENCV',
                                                               cdist_param=cdistpr)
            # Mask depth region where color picking failed
            invalid_mask = np.logical_not(valid_mask)
            depth[invalid_mask] = 0

            # Undistortion because Open3D expects undistorted RGBD
            mapped_color = cv2.undistort(mapped_color,
                                         param['depth']['K'], ddistpr)
            depth = pyrgbd.undistort_depth(depth, dfx, dfy, dcx, dcy,
                                           'OPENCV', ddistpr)
            # Median filter again after undisortion
            # Since undistortion algorithm is not good
            depth = pyrgbd.medianBlurForDepthWithNoHoleFilling(depth, 3)

            # Remove Background
            depth = background_removal(depth, background_removal_threshold)

            # Save mapped color
            #cv2.imwrite('mapped_{:05d}.png'.format(i), mapped_color)
            viz_depth = depth / 5.0
            viz_depth[viz_depth > 1.0] = 1.0
            viz_depth = (viz_depth * 255).astype(np.uint8)
            viz_depth = np.stack([viz_depth, viz_depth, viz_depth], axis=-1)
            mapped_color_with_depth = \
                cv2.addWeighted(mapped_color, 0.3, viz_depth, 0.7, 0)
            #cv2.imwrite('mapped_with_depth_{:05d}.png'.format(i),
            #            mapped_color_with_depth)

            pc, pc_color = pyrgbd.depth2pc(
                depth, dfx, dfy, dcx, dcy, mapped_color, keep_image_coord=False)
            pc_color = pc_color[:, [2, 1, 0]]  # BGR to RGB

            # Merge Multiple Kinects into
            # world_kinect coordinate (1st Kinect's coordinate)
            # TODO: Merge Multiple Kinects into panoptic_kinect coordinate
            pc = (param['d2w_R'] @ pc.T).T + param['d2w_t']

            #pyrgbd.write_pc_ply_txt('pc_{:05d}.ply'.format(i), pc, pc_color)
            global_pc += pc.tolist()
            global_pc_color += pc_color.tolist()

            # BGR to RGB
            # copy() for C-style memory allocation after fancy indexing
            mapped_color_bgr = mapped_color[..., [2, 1, 0]].copy()
            mapped_color_rgb = cv2.cvtColor(mapped_color_bgr, cv2.COLOR_BGR2RGB)

            makeDirectory(os.path.join(data_dir, 'kinectSync/text_image_list'))
            makeDirectory(os.path.join(data_dir, 'kinectSync/text_image_list/{:0=8}/'.format(frame_id)))

            mesh_filename = os.path.join(
                data_dir, 'kinectSync/text_image_list/50_{:02d}_{:08d}.jpg'.format(i+1, frame_id))
            cv2.imwrite(mesh_filename, mapped_color_rgb)

            o3d_color = o3d.geometry.Image(mapped_color_bgr)
            o3d_depth = o3d.geometry.Image(depth)
            rgbd = create_rgbd_image_from_color_and_depth(
                o3d_color, o3d_depth, depth_trunc=5.0, depth_scale=1.0,
                convert_rgb_to_intensity=False)

            h, w = depth.shape
            volume.integrate(
                rgbd,
                o3d.camera.PinholeCameraIntrinsic(
                    o3d.camera.PinholeCameraIntrinsic(w, h, dfx, dfy, dcx, dcy)),
                param['w2d_T'])

        global_pc = np.array(global_pc)
        global_pc_color = np.array(global_pc_color)

        makeDirectory(os.path.join(data_dir, 'kinectSync/point_cloud'))

        point_cloud_filename = os.path.join(
                data_dir, 'kinectSync/point_cloud/ptcloud_{:08d}.ply'.format(frame_id))

        pyrgbd.write_pc_ply_txt(point_cloud_filename, global_pc, global_pc_color)
        print(point_cloud_filename)

        mesh = volume.extract_triangle_mesh()
        mesh.compute_vertex_normals()

        makeDirectory(os.path.join(data_dir, 'kinectSync/meshes'))
        mesh_filename = os.path.join(
                data_dir, 'kinectSync/meshes/mesh_{:08d}.ply'.format(frame_id))
        o3d.io.write_triangle_mesh(mesh_filename, mesh, write_ascii = False)
