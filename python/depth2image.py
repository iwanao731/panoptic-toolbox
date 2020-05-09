import json
import cv2
import os
import sys
sys.path.append(os.path.abspath('.'))
from pyrgbd.util import *

def load_json(path):
	with open(path) as f:
		return json.load(f)

def loadCalibrationData(panopcalibFileName):
	panoptic_calibration = load_json(panopcalibFileName)
	return [camera for camera in panoptic_calibration['cameras'] if camera['type'] == 'kinect-color' ]

def makeDirectory(plyOutputDir):
	#print(('PLY files will be saved in: {0}').format(plyOutputDir));
	try:
		os.mkdir(plyOutputDir)
	except:
	    pass

if __name__ == '__main__':
	KINECT_NUM = 10
	root_path = '..'
	seqName = '171026_cello3'
	kinectImgDir = ('{0}/{1}/kinectImgs').format(root_path, seqName)
	kinectDepthDir = ('{0}/{1}/kinect_shared_depth').format(root_path, seqName)
	panopcalibFileName = ('{0}/{1}/calibration_{1}.json').format(root_path, seqName)

	# sync data	
	kinopSyncTableFileName = ('{0}/{1}/ksynctables_{1}.json').format(root_path, seqName)
	panopSyncTableFileName = ('{0}/{1}/synctables_{1}.json').format(root_path, seqName)
	kinopSyncTable = load_json(kinopSyncTableFileName)
	panopSyncTable = load_json(panopSyncTableFileName)

	# output
	OutputSyncDir = ('{0}/{1}/kinectSync').format(root_path, seqName)	
	makeDirectory(OutputSyncDir)	
	makeDirectory(OutputSyncDir + "/Color")	
	makeDirectory(OutputSyncDir + "/Depth")	

	start_frame = 500
	last_frame = 510
	frame_id_list = [i for i in range(start_frame, last_frame)]
	kinect_names = {str(id):('KINECTNODE{0}').format(str(id+1)) for id in range(10)}
	cameras = loadCalibrationData(panopcalibFileName)

	for frame_id in frame_id_list:
		selUnivTime = float(panopSyncTable['hd']['univ_time'][frame_id])
		for kinect_id in range(KINECT_NUM):
			cindex, dindex = checkSync(kinopSyncTable, kinect_names, selUnivTime, kinect_id)

			rgbFileName = ('{0}/{1}/{1}_{2}.jpg').format(kinectImgDir, cameras[kinect_id]['name'], str('{:0=8}'.format(cindex)))
			depthFileName = ('{0}/{1}/depthdata.dat').format(kinectDepthDir, kinect_names[str(kinect_id)])
			rgbim = cv2.imread(rgbFileName)
			depthim = readDepthIndex_1basedIdx(depthFileName, dindex)

			rgb_path = ('{0}/{1}/{2}').format(OutputSyncDir, "Color", cameras[kinect_id]['name'])
			depth_path = ('{0}/{1}/{2}').format(OutputSyncDir, "Depth", cameras[kinect_id]['name'])
			makeDirectory(rgb_path)
			makeDirectory(depth_path)

			out_rgb_fileName = ('{0}/{1}_{2}.jpg').format(rgb_path, cameras[kinect_id]['name'], str('{:0=8}'.format(frame_id)) )
			out_depth_fileName = ('{0}/{1}_{2}.png').format(depth_path, cameras[kinect_id]['name'], str('{:0=8}'.format(frame_id)) )

			print(out_rgb_fileName)
			print(out_depth_fileName)
			cv2.imwrite(out_rgb_fileName, rgbim)
			cv2.imwrite(out_depth_fileName, depthim)


