import json
import os
import array
import panutils
import numpy as np
import cv2
import open3d
from scipy.spatial.transform import Rotation

'''
def projectPoints(X, K, R, t, Kd):
    """ Projects points X (3xN) using camera intrinsics K (3x3),
    extrinsics (R,t) and distortion parameters Kd=[k1,k2,p1,p2,k3].
    
    Roughly, x = K*(R*X + t) + distortion
    
    See http://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html
    or cv2.projectPoints
    """
    
    x = np.asarray((R*X).transpose() + t)
    
    x[0:2,:] = x[0:2,:]/x[2,:]
    
    r = x[0,:]*x[0,:] + x[1,:]*x[1,:]
    
    x[0,:] = x[0,:]*(1 + Kd[0]*r + Kd[1]*r*r + Kd[4]*r*r*r) + 2*Kd[2]*x[0,:]*x[1,:] + Kd[3]*(r + 2*x[0,:]*x[0,:])
    x[1,:] = x[1,:]*(1 + Kd[0]*r + Kd[1]*r*r + Kd[4]*r*r*r) + 2*Kd[3]*x[0,:]*x[1,:] + Kd[2]*(r + 2*x[1,:]*x[1,:])

    x[0,:] = K[0,0]*x[0,:] + K[0,1]*x[1,:] + K[0,2]
    x[1,:] = K[1,0]*x[0,:] + K[1,1]*x[1,:] + K[1,2]
    
    return x

'''

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

def unprojectDepth_release(depth, camCalibData, bGenColorMap):
	k_dist = np.matrix(camCalibData['K_depth'])
	m_dist = np.matrix(camCalibData['M_depth'])[0:3,:]
	distCoeffs = np.array(camCalibData['distCoeffs_depth'])
	#print(k_depth)
	#print(m_depth)
	#print(distCoeffs_depth)
#	X, Y = np.meshgrid(np.arange(511), np.arange(423), sparse = True)
#	p2dd = [X, Y]
	return backprojection(depth, k_dist, m_dist, distCoeffs)

def makepointcloud(rgb, depth, camCalibData):
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
	#rgbd = create_from_color_and_depth(rgb, depth)
	depth =  open3d.geometry.Image(depth)
	color = open3d.geometry.Image(np.asanyarray(rgb, dtype=np.uint16))

	# need to make color data to be as same resolution as depth

	# Convert image to 3d point cloudS
#	rgbd =  open3d.geometry.create_rgbd_image_from_color_and_depth(depth, depth, convert_rgb_to_intensity = False)
#	pcd = open3d.geometry.create_point_cloud_from_rgbd_image(rgbd, intrinsic)

	pcd = open3d.geometry.create_point_cloud_from_depth_image(depth,intrinsic)
	return pcd

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
	projected_points_onto_rgb_image_plane = projectPoints(np.matrix(pcd.points).transpose(), np.matrix(camCalibData['K_color']), camera_pose, np.array(camCalibData['distCoeffs_color']))
	projected_points_onto_depth_image_plane = projectPoints(np.matrix(pcd.points).transpose(), np.matrix(camCalibData['K_depth']), camera_pose, np.array(camCalibData['distCoeffs_depth']))
	rgb_fit_depth = np.zeros((height,width,3), np.uint8)

#	R, t = decomposeRt(camera_pose)
#	projected_points_onto_rgb_image_plane = projectPoints(np.matrix(pcd.points).transpose(), np.matrix(camCalibData['K_color']), R, t, np.array(camCalibData['distCoeffs_color']))
#	projected_points_onto_depth_image_plane = projectPoints(np.matrix(pcd.points).transpose(), np.matrix(camCalibData['K_depth']),  R, t, np.array(camCalibData['distCoeffs_depth']))

	print(len(pcd.points))

	for i in range(len(pcd.points)):
		u_d = int(projected_points_onto_depth_image_plane[0,i])
		v_d = int(projected_points_onto_depth_image_plane[1,i])
		u_c = int(projected_points_onto_rgb_image_plane[0,i])
		v_c = int(projected_points_onto_rgb_image_plane[1,i])

		# if u_d < width and v_d < height and u_c < 1920 and v_c < 1280:
		#print(('{0} {1} {2} {3} {4}').format(u_d, v_d, u_c, v_c, i))
		rgb_fit_depth[v_d,u_d] = rgb[v_c,u_c]

	'''
	h = 424
	w = int(h * 1920/1080)
	rgb_resize = cv2.resize(rgb, (w, h))
	y = int(w/2 - 512/2)
	rgb_resize = rgb_resize[:,y:y+512]

	'''
	rgb_resize = cv2.resize(rgb, (512, 424))
	for u in range(width):
		for v in range(height):
			rgb_fit_depth[v,u] = rgb_resize[v, u]


	# cv2.imshow("", rgb_fit_depth)
	# cv2.waitKey(0)

	# need to make color data to be as same resolution as depth
	rgb_fit_depth = cv2.cvtColor(rgb_fit_depth, cv2.COLOR_BGR2RGB)

#	rgb = cv2.resize(rgb, (512, 424))
	color = open3d.geometry.Image(np.asanyarray(rgb_fit_depth, dtype=np.uint8))
	rgbd =  open3d.geometry.create_rgbd_image_from_color_and_depth(color, depth, convert_rgb_to_intensity = False)
	volume.integrate(rgbd, intrinsic, np.linalg.inv(camera_pose))

def backprojection(p2d, k_d, m_d, d_d):
	# https://towardsdatascience.com/inverse-projection-transformation-c866ccedef1c
	height = 512
	width = 424
	u0 = k_d[0,2]
	v0 = k_d[1,2]
	fx = k_d[0,0]
	fy = k_d[1,1]
	cam_points = np.zeros((height * width, 3))
	i = 0
	for v in range(width-1):
		for u in range(height-1):
			x = (u - u0) * p2d[v, u] / fx
			y = (v - v0) * p2d[v, u] / fy
			z = p2d[v, u]
			cam_points[i] = (x, y, z)
			i += 1
	return cam_points

def unproject( p2d, k_d, m_d, d_d ):	
	# Normalized point  (TBD)
	b = np.concatenate([p2d[:,0:2].transpose(), np.ones((1, 424))])
	pn2d = np.matrix(np.linalg.lstsq(k_d, b, rcond=None))
	
	#print(pn2d[0,3])
	#k = [reshape(distCoeffs[0:5],[],1); zeros(12-5,1)];
	x0 = pn2d[0,0][:,0]
	y0 = pn2d[0,3][:,0]
#	print(x0, y0)
	x = x0; y = y0;

	# Undistortion iterations. (TBD)
	for iter in range(5):
		r2 = np.dot(x,x) + np.dot(y,y)

	p3d = p2d

	# Calculate 3D Point
	m_d_inv = np.linalg.inv(np.concatenate([m_d, np.matrix([0,0,0,1])]))
	v = np.transpose(np.ones(4))
	#print(p3d)
	#np.concatenate(p3d, v)
	#print(p3d.shape)

#	dist = np.concatenate(m_d, np.array([0,0,0,1]))
#	print(dist.shape)


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

# parameter setting
start_frame = 500
last_frame = 510	
bVisOutput = False # Turn on, if you want to visualize

## Pur your root pahth where sequence folders are located
hd_index_list = [i for i in range(start_frame, last_frame)]

#============================
# DATA LOADING
#============================

# load sync tables
with open(syncTableFileName) as sfile: #Panoptic Sync Tables
    ksync = json.load(sfile)

knames = {str(id):('KINECTNODE{0}').format(str(id)) for id in range(1, 10+1)}
'''
knames = [('KINECTNODE{0}').format(id) for id in range(1, 10)]
print(knames[0])
'''

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
#	for idk in range(1, 2): #kinect x 10
		# select correspondence frame index rgb and depth
		# color
		c_univ_time = np.array(ksync['kinect']['color'][knames[str(idk)]]['univ_time']) 
		diff_time = abs(selUnivTime - (c_univ_time - 6.25)) # Why original code is minus 6.25?
		cindex = np.argmin(diff_time)
		time_distc = diff_time[cindex]

		# depth
		d_univ_time = np.array(ksync['kinect']['depth'][knames[str(idk)]]['univ_time']) 
		diff_time = abs(selUnivTime - d_univ_time)
		dindex = np.argmin(diff_time)
		time_distd = diff_time[dindex]

		# Filtering if current kinect data is far from the selected time
		val = abs(d_univ_time[dindex] - c_univ_time[cindex])
		if abs(val) > 6.5:
			print('Skipping {0}, depth-color diff {1}\n'.format(idk, val))

		if time_distc > 30 or time_distd > 17:
			print('Skipping {0}\n'.format(idk))

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
		
		#point3d = unprojectDepth_release(depthim, camCalibData, True)
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








	# open3d.visualization.draw_geometries([mesh])

		# pcd = makepointcloud(rgbim, depthim, camCalibData)

		'''
		T_kinectColor2PanopticWorld = np.linalg.inv(mat4)

		scale_kinoptic2panoptic = np.identity(4);
		scaleFactor = 100; # %0.01; %centimeter to meter
		scale_kinoptic2panoptic[0:3,0:3] = scaleFactor*scale_kinoptic2panoptic[0:3,0:3];

		#R, t = decomposeRt(mat4)
		T_kinectColor2KinectLocal = np.matrix(kinect_calibration['sensors'][idk-1]['M_color'])
		T_kinectLocal2KinectColor = np.linalg.inv(T_kinectColor2KinectLocal)
		T_kinectLocal2PanopticWorld = (T_kinectColor2PanopticWorld.dot(scale_kinoptic2panoptic)).dot(T_kinectLocal2KinectColor)

		for i, pt in enumerate(np.asarray(pcd.points)):
			pt = T_kinectLocal2PanopticWorld.dot((np.append(pt, 1.0)).transpose())
			pcd.points[i][0] = pt[0,0]
			pcd.points[i][1] = pt[0,1]
			pcd.points[i][2] = pt[0,2]

		#point3d_panopticWorld = *[point3d
		#for i, pt in enumerate(pcd.points):
		#	pcd.points[i] = R.dot(pt) + t/1000

		try:
			ptc_path = ('{0}/{1}').format(plyOutputDir, knames[str(idk)])
			os.mkdir(ptc_path)
		except:
		    pass

		val = str(('{:0=8}').format(hd_index_afterOffest))
		name = ('{0}/ptcloud_{1}.ply').format(ptc_path, val)
		open3d.io.write_point_cloud(name, pcd)

		## filtering 

		# Project 3d points (from depth) to color image

		# extract color for dept camera

		# Transform Kinect Local to Panoptic World

		# Merge Multiple Kinect into panoptic_kinect coordinate
		'''

	# Delete floor light

	# Save point cloud as ply file
	'''
	'''









