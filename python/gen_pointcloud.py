import json
import array
import panutils
import numpy as np
import cv2
import open3d
from scipy.spatial.transform import Rotation
from pyrgbd.util import *

#============================
# Input/Output Path Setting
#============================

# Input/Output Path Setting  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
# The following folder hiearchy is assumed:
# (root_path)/(seqName)/kinectImgs
# (root_path)/(seqName)/kcalibration_(seqName).json
# (root_path)/(seqName)/ksynctables_(seqName).json
# (root_path)/(seqName)/calibration_(seqName).json
# (root_path)/(seqName)/synctables_(seqName).json

root_path = '..'
seqName = '171026_cello3'

# parameter setting
start_frame = 500
last_frame = 510	
bVisOutput = False # Turn on, if you want to visualize

#Relative Paths
kinectImgDir = ('{0}/{1}/kinectImgs').format(root_path, seqName)
kinectDepthDir = ('{0}/{1}/kinect_shared_depth').format(root_path, seqName)
calibFileName = ('{0}/{1}/kcalibration_{1}.json').format(root_path, seqName)
syncTableFileName = ('{0}/{1}/ksynctables_{1}.json').format(root_path, seqName)
panopcalibFileName = ('{0}/{1}/calibration_{1}.json').format(root_path, seqName)
panopSyncTableFileName = ('{0}/{1}/synctables_{1}.json').format(root_path, seqName)

# Output folder Path
#Change the following if you want to save outputs on another folder
plyOutputDir = ('{0}/{1}/kinoptic_ptclouds').format(root_path, seqName)
print(('PLY files will be saved in: {0}').format(plyOutputDir));
try:
	os.mkdir(plyOutputDir)
except:
    pass

## Pur your root pahth where sequence folders are located
hd_index_list = [i for i in range(start_frame, last_frame)]

#============================
# DATA LOADING
#============================

# load sync tables
with open(syncTableFileName) as sfile: #Panoptic Sync Tables
    ksync = json.load(sfile)

knames = {str(id):('KINECTNODE{0}').format(str(id)) for id in range(1, 10+1)}

with open(panopSyncTableFileName) as psfile: #Panoptic Sync Tables
    psync = json.load(psfile)

# load kinect calibration file
with open(calibFileName) as cfile:
    kinect_calibration = json.load(cfile)

with open(panopcalibFileName) as pcfile: 
    panoptic_calibration = json.load(pcfile)
    panoptic_camNames = [str(camera['name']) for camera in panoptic_calibration['cameras'] if camera['type'] == 'kinect-color' ]
    panoptic_cameras = [camera for camera in panoptic_calibration['cameras'] if camera['type'] == 'kinect-color' ]

#============================
# POINT CLOUD SUMMERIZATION
#============================
bFirst = True
for hd_index in hd_index_list:
	# output path setting
	hd_index_afterOffest = hd_index
	out_fileName = ('{0}/ptcloud_hd{1}.ply').format(plyOutputDir, str('{:0=8}'.format(hd_index_afterOffest)) )

	# compute universal time
	selUnivTime = float(psync['hd']['univ_time'][hd_index])
	print('hd_index: {0}, UnivTime: {1}'.format(hd_index, selUnivTime))

	#============================
	# LOOP EACH KINECT
	#============================
	# point cloud output from all kinects
	all_point3d_panopticWorld = []

	# colors for point cloud 
	all_colorsv = open3d.PointCloud()

	# Iterating kinects. Change this if you want a subpart
	for idk in range(1, len(panoptic_camNames) + 1): #kinect x 10
		# select correspondence frame index rgb and depth
			cindex, dindex = checkSync(ksync, knames, selUnivTime, idk)

		# extract image and depth
		rgbFileName = ('{0}/{1}/{1}_{2}.jpg').format(kinectImgDir, panoptic_camNames[idk-1], str('{:0=8}'.format(cindex)))
		depthFileName = ('{0}/{1}/depthdata.dat').format(kinectDepthDir, knames[str(idk)])
		rgbim = cv2.imread(rgbFileName)
		depthim = readDepthIndex_1basedIdx(depthFileName, dindex)

		# check valid pixel (mask)
		#===========
		# TBD
		#===========

		volume = open3d.integration.ScalableTSDFVolume(
			voxel_length=1.0 / 512.0,
			sdf_trunc=0.04,
			color_type=open3d.integration.TSDFVolumeColorType.RGB8)

		# show images (RGB, DEPTH, MASK)
		if bVisOutput:
			# color
			if rgbim is None:
				print(('loading error: {0}').format(rgbFileName))
			else:
				resized_img = cv2.resize(rgbim,(960, 540))
				cv2.imshow("image", resized_img)
				#open3d.visualization.draw_geometries([mesh])

			# depth
			cv2.imshow(('depth').format(hd_index, idk), depthim)
			#cv2.imshow('depth', depthim)
			if bFirst == True:
				cv2.waitKey(0)		
				bFirst = False

		# back project depth to 3d point?
		camCalibData = kinect_calibration['sensors'][idk-1]
		
		R = np.matrix(panoptic_cameras[idk-1]['R'])
		t = np.matrix(panoptic_cameras[idk-1]['t'])
		camera_pose = composeMat4(R, t)
		makevolume(rgbim, depthim, camera_pose, camCalibData, volume)

		mesh = volume.extract_triangle_mesh()
		mesh.compute_vertex_normals()

		try:
			ptc_path = ('{0}/{1}').format(plyOutputDir, knames[str(idk)])
			os.mkdir(ptc_path)
		except:
		    pass
		val = str(('{:0=8}').format(hd_index_afterOffest))
		name = ('{0}/ptcloud_{1}.ply').format(ptc_path, val)
		open3d.io.write_triangle_mesh(name, mesh)

		## filtering 

		# Project 3d points (from depth) to color image

		# extract color for dept camera

		# Transform Kinect Local to Panoptic World

		# Merge Multiple Kinect into panoptic_kinect coordinate
		'''

	# Delete floor light

	# Save point cloud as ply file
	'''