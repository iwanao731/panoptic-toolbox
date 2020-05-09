import json
import os
import numpy as np
import sys
sys.path.append(os.path.abspath('.'))
from scipy.spatial.transform import Rotation

def load_json(path):
    with open(path) as f:
        return json.load(f)

def convert2GL():
	# mat4 = np.eye(4)
	# rotvec = np.array([np.pi, 0, 0])
	# mat4[0:3,0:3] = (Rotation.from_rotvec(rotvec)).as_dcm()	
	# return mat4

	rotvecX = np.array([np.pi, 0, 0])
	rotvecY = np.array([0, np.pi, 0])
	rotvecZ = np.array([0, 0, np.pi])
	return (Rotation.from_rotvec(rotvecY)).as_dcm() @ (Rotation.from_rotvec(rotvecZ)).as_dcm()

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
    w2d_T = d_T @ np.array(kinect['M_world2sensor'])
    d2w_T = np.linalg.inv(w2d_T)

    d2w_T[0:3, 0:3] = d2w_T[0:3, 0:3] @ convert2GL()
    w2d_T = np.linalg.inv(d2w_T)

    # w2d_T = convert2GL() @ d_T @ np.array(kinect['M_world2sensor'])
    # d2w_T = np.linalg.inv(w2d_T)

    rotvec = np.array([np.pi, 0, 0])
    param['w2d_R'] = w2d_T[0:3, 0:3]

    param['w2d_t'] = w2d_T[0:3, 3]
    param['w2d_T'] = w2d_T

    param['d2w_R'] = d2w_T[0:3, 0:3]
    param['d2w_t'] = d2w_T[0:3, 3]
    param['d2w_T'] = d2w_T

    return param

if __name__ == '__main__':

	KINECT_NUM = 10
	data_dir = '../171026_cello3/'
	kinect_params = load_json(os.path.join(data_dir,
	                               'Kcalibration_171026_cello3.json'))

	f = open(os.path.join(data_dir,
	                               'bundle.rd.out'), 'w') # 書き込みモードで開く
	f.write('# Bundle file v0.3\n')
	f.write(str(KINECT_NUM) + ' 0\n')

	for kinect_id in range(KINECT_NUM):
		param = parse_camera_params(kinect_params['sensors'][kinect_id])
		fxy = float(param['color']['fx'] ) 
		k1 = param['color']['distCoeffs'][0]
		k2 = param['color']['distCoeffs'][1]
		str1 = '{0} {1} {2}\n'.format(fxy, k1, k2)
		f.write(str1)
		
		for i in range(3):
			for j in range(3):
				if j == 2:
					f.write(str(param['w2d_R'][i, j]) + '\n')
				else:
					f.write(str(param['w2d_R'][i, j]) + ' ')

		for i in range(3):
			if i == 2:
				str3 = str(param['w2d_t'][i]) + '\n' 
			else:
				str3 = str(param['w2d_t'][i]) + ' ' 

			f.write(str3)
		
	f.close() # ファイルを閉じる

