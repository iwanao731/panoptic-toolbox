#!/usr/bin/env python

import sys
import os
import subprocess

def make_batch_script(index):
    filter_script_mlx = """
    meshlabserver.exe -p ../meshlab_projects/meshlab_{0}.mlp -w ../meshlab_projects/meshlab_{0}_.mlp -s ../meshlab_parameters/image_registration.mlx
    meshlabserver.exe -p ../meshlab_projects/meshlab_{0}_.mlp -w ../meshlab_projects/meshlab_{0}__.mlp -s ../meshlab_parameters/parameterize_texturing.mlx
    meshlabserver -i ../meshes/mesh_{0}_out_out.ply -o ../meshes/mesh_{0}_out.obj -m wt vn

    REM move files
    move ../meshes/mesh_{0}_out.obj ./
    move ../meshes/mesh_{0}_out.obj.mtl ./mesh_{0}_out.obj.mtl

    REM remove projects
    move ../meshlab_projects/meshlab_{0}_.mlp ./
    del meshlab_{0}_.mlp
    move ../meshlab_projects/meshlab_{0}__.mlp ./
    del meshlab_{0}__.mlp
    move ../meshes/mesh_{0}_out.ply ./
    del mesh_{0}_out.ply
    move ../meshes/mesh_{0}_out_out.ply ./
    del mesh_{0}_out_out.ply

    REM rename
    rename mesh_{0}_out.obj mesh_{0}.obj
    rename texture.png texture_{0}.png

    REM remove trash
    del rendering.jpg
    del mesh_{0}_out.obj.mtl
    """.format(str('{:0=8}'.format(index)))
    return filter_script_mlx

def create_all_batch_file(start_frame, last_frame, filename):
  with open(filename, 'w') as f:
    frame_id_list = [i for i in range(start_frame, last_frame)]
    all_list = ""
    for frame_id in frame_id_list:
        all_list += str('call texture_map_{:0=8}.bat\n'.format(frame_id))
    f.write(all_list)

def create_batch_file(index):
  filename = str('../171026_cello3/kinectSync/textured_meshes/texture_map_{:0=8}.bat'.format(index))
  with open(filename, 'w') as f:
    f.write(make_batch_script(index))
    print(filename)

if __name__ == '__main__':
  start_frame = 400
  last_frame = 510
  frame_id_list = [i for i in range(start_frame, last_frame)]
  for frame_id in frame_id_list:
    create_batch_file(frame_id)
  create_all_batch_file(start_frame, last_frame, "../171026_cello3/kinectSync/textured_meshes/texture_mapping_all.bat")
