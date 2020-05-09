import bpy
import json
from scipy.spatial.transform import Rotation
import numpy as np

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
    mat4[0][0] = R[0][0]
    mat4[0][1] = R[0][1]
    mat4[0][2] = R[0][2]
    mat4[1][0] = R[1][0]
    mat4[1][1] = R[1][1]
    mat4[1][2] = R[1][2]
    mat4[2][0] = R[2][0]
    mat4[2][1] = R[2][1]
    mat4[2][2] = R[2][2]
    mat4[0][3] = t[0]
    mat4[1][3] = t[1]
    mat4[2][3] = t[2]
    mat4[3][3] = 1.0
    return mat4
    
dir_path = "D:/Projects/VolumetricVideo/data/panoptic-toolbox/171204_pose1_sample/"

filepath = dir_path + "calibration_171204_pose1_sample.json"
with open(filepath) as f:
    df = json.load(f)
    for cam in df['cameras']:
        if cam['type'] == 'hd':
            t = (cam['t'][0][0], cam['t'][1][0], cam['t'][2][0])        
            mat4 = composeMat4(cam['R'], t)
            mat4inv = np.linalg.inv(mat4)
            R, t = decomposeRt(mat4inv)
            t = (t[0], -t[2], -t[1])
            rot = Rotation.from_dcm(R)            
            re = rot.as_euler('xyz')
            print(re)
            bpy.ops.object.camera_add(align='WORLD', location = t, rotation = (re[0], 3.14/2, 0))
            '''
            bpy.ops.object.camera_add(align='WORLD', location = t, rotation = (-re[0], re[1], re[2]))
            bpy.ops.object.camera_add(align='WORLD', location = t, rotation = (re[0], -re[1], re[2]))
            bpy.ops.object.camera_add(align='WORLD', location = t, rotation = (re[0], re[1], -re[2]))
            
            bpy.ops.object.camera_add(align='WORLD', location = t, rotation = (re[0], re[1], re[2]))
            bpy.ops.object.camera_add(align='WORLD', location = t, rotation = (re[0], re[1], re[2]))
            bpy.ops.object.camera_add(align='WORLD', location = t, rotation = (-re[1], re[2], re[0]))
            bpy.ops.object.camera_add(align='WORLD', location = t, rotation = (-re[1], re[2], re[0]))
            bpy.ops.object.camera_add(align='WORLD', location = t, rotation = (-re[1], re[2], re[0]))
            #bpy.ops.object.camera_add(align='WORLD', location = t, rotation = (-re[1], re[2], re[0]))
            '''
            bpy.context.object.data.type = 'ORTHO'
