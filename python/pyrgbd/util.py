import numpy as np
import cv2
import json
import array

def background_removal(depth_image, depth_threshold):
    removal_depth = depth_image.copy()
    h, w = depth_image.shape
    for u in range(h):
        for v in range(w):
            if depth_threshold < depth_image[u, v]:
                removal_depth[u, v] = 0.0
    return removal_depth

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
    
def makeDirectory(plyOutputDir):
    try:
        os.mkdir(plyOutputDir)
    except:
        pass

def load_json(path):
    with open(path) as f:
        return json.load(f)

def loadCalibrationData(panopcalibFileName):
    panoptic_calibration = load_json(panopcalibFileName)
    return [camera for camera in panoptic_calibration['cameras'] if camera['type'] == 'kinect-color' ]

def checkSync(kinect_sync_table, kinect_name_list, universal_time, kinect_id):
    c_univ_time = np.array(kinect_sync_table['kinect']['color'][kinect_name_list[str(kinect_id)]]['univ_time']) 
    diff_time = abs(universal_time - (c_univ_time - 6.25)) # Why original code is minus 6.25?
    cindex = np.argmin(diff_time)
    time_distc = diff_time[cindex]

    # depth
    d_univ_time = np.array(kinect_sync_table['kinect']['depth'][kinect_name_list[str(kinect_id)]]['univ_time']) 
    diff_time = abs(universal_time - d_univ_time)
    dindex = np.argmin(diff_time)
    time_distd = diff_time[dindex]

    # Filtering if current kinect data is far from the selected time
    val = abs(d_univ_time[dindex] - c_univ_time[cindex])
    if abs(val) > 6.5:
        print('Skipping {0}, depth-color diff {1}\n'.format(kinect_id, val))

    if time_distc > 30 or time_distd > 17:
        print('Skipping {0}\n'.format(kinect_id))

    return cindex, dindex    

# https://docs.opencv.org/4.3.0/d9/d0c/group__calib3d.html
def _undistort_pixel_opencv(u, v, fx, fy, cx, cy, k1, k2, p1, p2,
                            k3=0.0, k4=0.0, k5=0.0, k6=0.0):

    u1 = (u - cx) / fx
    v1 = (v - cy) / fy
    u2 = u1 ** 2
    v2 = v1 ** 2
    r2 = u2 + v2

    # https://github.com/egonSchiele/OpenCV/blob/master/modules/imgproc/src/undistort.cpp#L133
    _2uv = 2 * u1 * v1
    kr = (1 + ((k3*r2 + k2)*r2 + k1)*r2)/(1 + ((k6*r2 + k5)*r2 + k4)*r2)
    u_ = fx*(u1 * kr + p1 * _2uv + p2 * (r2+2*u2)) + cx
    v_ = fy*(v1 * kr + p1 * (r2 + 2*v2) + p2 * _2uv) + cy

    return u_, v_


def undistort_pixel(u, v, fx, fy, cx, cy, distortion_type, distortion_param):
    if distortion_type == "OPENCV" and 4 <= len(distortion_param) <= 8:
        # k1, k2, p1, p2 = distortion_param
        return _undistort_pixel_opencv(u, v, fx, fy,
                                       cx, cy, *tuple(distortion_param))
    raise NotImplementedError(
        distortion_type + " with param " + distortion_param
        + " is not implemented")


def undistort_depth(depth, fx, fy, cx, cy, distortion_type, distortion_param):
    # Undistortion for depth image
    # cv2.undistort uses bilinar interpolation
    # but it causes artifacts for boundary of depth image.
    # Nearest Neighbor is preferred.
    # This function provides NN like undistortion

    undistorted = np.zeros_like(depth)
    h, w = depth.shape
    u = np.tile(np.arange(w), (h, 1))
    v = np.tile(np.arange(h), (w, 1)).T
    u, v = undistort_pixel(u, v, fx, fy, cx, cy,
                           distortion_type, distortion_param)
    v, u = np.rint(v).astype(np.int), np.rint(u).astype(np.int)

    # Make valid mask
    v_valid = np.logical_and(0 <= v, v < h)
    u_valid = np.logical_and(0 <= u, u < w)
    uv_valid = np.logical_and(u_valid, v_valid)
    uv_invalid = np.logical_not(uv_valid)

    # Fiil stub
    v[v < 0] = 0
    v[(h - 1) < v] = h - 1
    u[u < 0] = 0
    u[(w - 1) < u] = w - 1

    # Copy depth value
    # Similar to Nearest Neighbor
    undistorted[v, u] = depth

    # 0 for invalid
    undistorted[uv_invalid] = 0

    return undistorted


def medianBlurForDepthWithNoHoleFilling(depth, ksize):
    # cv2.medianBlur may fill invalid (0) for depth image
    # This function prevents it
    invalid_mask = depth == 0
    depth = cv2.medianBlur(depth, ksize)
    depth[invalid_mask] = 0
    return depth


def unproject(u, v, d, fx, fy, cx, cy):
    x = (u - cx) * d / fx
    y = (v - cy) * d / fy
    # for scalar and tensor
    return np.stack([x, y, d], axis=-1)


def project(x, y, z, fx, fy, cx, cy, with_depth=True):
    u = x * fx / z + cx
    v = y * fy / z + cy
    # for scalar and tensor
    if with_depth:
        return np.stack([u, v, z], axis=-1)
    return np.stack([u, v], axis=-1)


# TODO: faster version by tensor operation
def depth2pc_naive(depth, fx, fy, cx, cy, color=None, ignore_zero=True,
                   distortion_type=None, distortion_param=[],
                   distortion_interp='NN'):
    if depth.ndim != 2:
        raise ValueError()
    with_color = color is not None
    with_distortion = distortion_type is not None
    pc = []
    pc_color = []
    h, w = depth.shape
    for v in range(h):
        for u in range(w):
            d = depth[v, u]
            if ignore_zero and d <= 0:
                continue
            if with_distortion:
                u, v = undistort_pixel(u, v, distortion_type, distortion_param)
            p = unproject(u, v, d, fx, fy, cx, cy)
            pc.append(p)
            if with_color:
                # interpolation for float uv by undistort_pixel
                if with_distortion and distortion_interp == 'NN':
                    v, u = np.rint(v), np.rint(u)
                elif with_distortion:
                    raise NotImplementedError('distortion_interp ' +
                                              distortion_interp +
                                              ' is not implemented')
                if v < 0:
                    v = 0
                elif h - 1 < v:
                    v = h - 1
                if u < 0:
                    u = 0
                elif w - 1 < u:
                    u = w - 1
                pc_color.append(color[v, u])

    pc = np.array(pc)
    if with_color:
        pc_color = np.array(pc_color)
    return pc, pc_color


def depth2pc(depth, fx, fy, cx, cy, color=None, ignore_zero=True,
             keep_image_coord=False,
             distortion_type=None, distortion_param=[],
             distortion_interp='NN'):
    if depth.ndim != 2:
        raise ValueError()
    with_color = color is not None
    with_distortion = distortion_type is not None
    if ignore_zero:
        valid_mask = depth > 0
    else:
        valid_mask = np.ones(depth.shape, dtype=np.bool)
    invalid_mask = np.logical_not(valid_mask)
    h, w = depth.shape
    u = np.tile(np.arange(w), (h, 1))
    v = np.tile(np.arange(h), (w, 1)).T
    if with_distortion:
        u, v = undistort_pixel(u, v, fx, fy, cx, cy,
                               distortion_type, distortion_param)
    pc = unproject(u, v, depth, fx, fy, cx, cy)
    pc[invalid_mask] = 0

    pc_color = None
    if with_color:
        # interpolation for float uv by undistort_pixel
        if with_distortion and distortion_interp == 'NN':
            v, u = np.rint(v), np.rint(u)
        elif with_distortion:
            raise NotImplementedError('distortion_interp ' +
                                      distortion_interp +
                                      ' is not implemented')
        v[v < 0] = 0
        v[(h - 1) < v] = h - 1
        u[u < 0] = 0
        u[(w - 1) < u] = w - 1
        pc_color = color[v, u]
        pc_color[invalid_mask] = 0

    # return as organized point cloud keeping original image shape
    if keep_image_coord:
        return pc, pc_color

    # return as a set of points
    return pc[valid_mask], pc_color[valid_mask]


def _make_ply_txt(pc, color, normal):
    header_lines = ["ply", "format ascii 1.0",
                    "element vertex " + str(len(pc)),
                    "property float x", "property float y", "property float z"]
    has_normal = len(pc) == len(normal)
    has_color = len(pc) == len(color)
    if has_normal:
        header_lines += ["property float nx",
                         "property float ny", "property float nz"]
    if has_color:
        header_lines += ["property uchar red", "property uchar green",
                         "property uchar blue", "property uchar alpha"]
    # no face
    header_lines += ["element face 0",
                     "property list uchar int vertex_indices", "end_header"]
    header = "\n".join(header_lines) + "\n"

    data_lines = []
    for i in range(len(pc)):
        line = [pc[i][0], pc[i][1], pc[i][2]]
        if has_normal:
            line += [normal[i][0], normal[i][1], normal[i][2]]
        if has_color:
            line += [int(color[i][0]), int(color[i][1]), int(color[i][2]), 255]
        line_txt = " ".join([str(x) for x in line])
        data_lines.append(line_txt)
    data_txt = "\n".join(data_lines)

    # no face
    ply_txt = header + data_txt

    return ply_txt


def write_pc_ply_txt(path, pc, color=[], normal=[]):
    with open(path, 'w') as f:
        txt = _make_ply_txt(pc, color, normal)
        f.write(txt)
