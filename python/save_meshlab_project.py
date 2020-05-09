import sys
import os
import subprocess

def make_meshlab_script(index):
  filter_script_mlx = '''<!DOCTYPE MeshLabDocument>
  <MeshLabProject>
   <MeshGroup>
    <MLMesh label="mesh_{0}.ply" visible="1" filename="../meshes/mesh_{0}.ply">
     <MLMatrix44>
1 0 0 0 
0 1 0 0 
0 0 1 0 
0 0 0 1 
</MLMatrix44>
     <RenderingOption pointColor="131 149 69 255" pointSize="3" boxColor="234 234 234 255" wireColor="64 64 64 255" solidColor="192 192 192 255" wireWidth="1">100001010000000000000100000001011010001010100000000100111010100000001001</RenderingOption>
    </MLMesh>
   </MeshGroup>
   <RasterGroup>
    <MLRaster label="50_01_{0}.jpg">
     <VCGCamera FocalMm="1055.17" LensDistortion="0 0" CenterPx="960 540" RotationMatrix="-0.999967 0.00329218 -0.00742563 0 0.00327491 0.999992 0.00233614 0 0.00743326 0.00231175 -0.99997 0 0 0 0 1 " ViewportPx="1920 1080" CameraType="0" PixelSizeMm="1 1" TranslationVector="0.0601967 0.009643 -0.0486471 1" BinaryData="0"/>
     <Plane semantic="1" fileName="../color/50_01/50_01_{0}.jpg"/>
    </MLRaster>
    <MLRaster label="50_02_{0}.jpg">
     <VCGCamera FocalMm="1071.2" LensDistortion="0 0" CenterPx="960 540" RotationMatrix="-0.328458 0.00227727 -0.944516 0 -0.015026 0.999858 0.00763601 0 0.944399 0.0167004 -0.328377 0 0 0 0 1 " ViewportPx="1920 1080" CameraType="0" PixelSizeMm="1 1" TranslationVector="-2.64118 0.0386688 -1.80414 1" BinaryData="0"/>
     <Plane semantic="1" fileName="../color/50_02/50_02_{0}.jpg"/>
    </MLRaster>
    <MLRaster label="50_03_{0}.jpg">
     <VCGCamera FocalMm="1060.75" LensDistortion="0 0" CenterPx="960 540" RotationMatrix="0.805049 -0.00739972 -0.593162 0 0.0169195 0.999802 0.0104909 0 0.592967 -0.0184817 0.805015 0 0 0 0 1 " ViewportPx="1920 1080" CameraType="0" PixelSizeMm="1 1" TranslationVector="-1.77119 -0.0322246 -4.82635 1" BinaryData="0"/>
     <Plane semantic="1" fileName="../color/50_03/50_03_{0}.jpg"/>
    </MLRaster>
    <MLRaster label="50_04_{0}.jpg">
     <VCGCamera FocalMm="1032.92" LensDistortion="0 0" CenterPx="960 540" RotationMatrix="0.819135 0.0166216 0.57336 0 -0.0452182 0.99834 0.0356597 0 -0.571816 -0.0551365 0.818527 0 0 0 0 1 " ViewportPx="1920 1080" CameraType="0" PixelSizeMm="1 1" TranslationVector="1.30523 -0.0187081 -4.94388 1" BinaryData="0"/>
     <Plane semantic="1" fileName="../color/50_04/50_04_{0}.jpg"/>
    </MLRaster>
    <MLRaster label="50_05_{0}.jpg">
     <VCGCamera FocalMm="1009.68" LensDistortion="0 0" CenterPx="960 540" RotationMatrix="-0.25137 -0.0281328 0.967482 0 -0.0390093 0.99906 0.0189157 0 -0.967104 -0.032986 -0.252231 0 0 0 0 1 " ViewportPx="1920 1080" CameraType="0" PixelSizeMm="1 1" TranslationVector="2.33023 -0.0092905 -2.04543 1" BinaryData="0"/>
     <Plane semantic="1" fileName="../color/50_05/50_05_{0}.jpg"/>
    </MLRaster>
    <MLRaster label="50_06_{0}.jpg">
     <VCGCamera FocalMm="941.566" LensDistortion="0 0" CenterPx="960 540" RotationMatrix="-0.854572 0.0367953 0.518027 0 0.332515 0.804984 0.491361 0 -0.398924 0.592155 -0.700152 0 0 0 0 1 " ViewportPx="1920 1080" CameraType="0" PixelSizeMm="1 1" TranslationVector="1.12998 -1.25964 -0.867173 1" BinaryData="0"/>
     <Plane semantic="1" fileName="../color/50_06/50_06_{0}.jpg"/>
    </MLRaster>
    <MLRaster label="50_07_{0}.jpg">
     <VCGCamera FocalMm="968.922" LensDistortion="0 0" CenterPx="960 540" RotationMatrix="-0.76999 0.0434489 -0.636575 0 -0.31776 0.839044 0.441626 0 0.553302 0.542326 -0.63225 0 0 0 0 1 " ViewportPx="1920 1080" CameraType="0" PixelSizeMm="1 1" TranslationVector="-1.39385 -1.40344 -0.97512 1" BinaryData="0"/>
     <Plane semantic="1" fileName="../color/50_07/50_07_{0}.jpg"/>
    </MLRaster>
    <MLRaster label="50_08_{0}.jpg">
     <VCGCamera FocalMm="1081.4" LensDistortion="0 0" CenterPx="960 540" RotationMatrix="0.385198 0.0779841 -0.919533 0 -0.546362 0.822292 -0.159137 0 0.743714 0.563698 0.359353 0 0 0 0 1 " ViewportPx="1920 1080" CameraType="0" PixelSizeMm="1 1" TranslationVector="-2.28592 -1.62354 -3.53288 1" BinaryData="0"/>
     <Plane semantic="1" fileName="../color/50_08/50_08_{0}.jpg"/>
    </MLRaster>
    <MLRaster label="50_09_{0}jpg">
     <VCGCamera FocalMm="1056" LensDistortion="0 0" CenterPx="960 540" RotationMatrix="0.994911 0.088651 0.047898 0 -0.042414 0.799635 -0.598987 0 -0.0914017 0.593907 0.799325 0 0 0 0 1 " ViewportPx="1920 1080" CameraType="0" PixelSizeMm="1 1" TranslationVector="-0.107781 -1.63185 -5.02282 1" BinaryData="0"/>
     <Plane semantic="1" fileName="../color/50_09/50_09_{0}.jpg"/>
    </MLRaster>
    <MLRaster label="50_10_{0}.jpg">
     <VCGCamera FocalMm="1005.23" LensDistortion="0 0" CenterPx="960 540" RotationMatrix="0.238293 0.0233162 0.970913 0 0.588901 0.791485 -0.163542 0 -0.772277 0.610743 0.174874 0 0 0 0 1 " ViewportPx="1920 1080" CameraType="0" PixelSizeMm="1 1" TranslationVector="2.00881 -1.56284 -3.34334 1" BinaryData="0"/>
     <Plane semantic="1" fileName="../color/50_10/50_10_{0}.jpg"/>
    </MLRaster>
   </RasterGroup>
  </MeshLabProject>
  '''.format(str('{:0=8}'.format(index)))
  return filter_script_mlx

#cwd = os.getcwd()

def create_project_file(index):
  filename = str('../171026_cello3/kinectSync/meshlab_projects/meshlab_{:0=8}.mlp'.format(index))
  with open(filename, 'w') as f:
    f.write(make_meshlab_script(index))
  return filename

if __name__ == '__main__':
  start_frame = 400
  last_frame = 510
  frame_id_list = [i for i in range(start_frame, last_frame)]
  for frame_id in frame_id_list:
    create_project_file(frame_id)
