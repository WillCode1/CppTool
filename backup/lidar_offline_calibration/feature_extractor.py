#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import pandas as pd
import os
import numpy as np
import glob
import json
import matplotlib.cm
import matplotlib.colors
import matplotlib.pyplot as plt
import open3d as o3d
import argparse
from scipy.spatial.transform import Rotation as R
from skimage.measure import ransac, LineModelND, CircleModel
from mpl_toolkits.mplot3d import Axes3D
from numpy import array, cross
from numpy.linalg import solve, norm
from math import sqrt
from ocamcamera import OcamCamera
cv = cv2

#input directory
IMG_DIRECTORY = "./sample_data_2021_07_16/image"
PC_DIRECTORY = "./sample_data_2021_07_16/pcd"

#Sampling and Pairing
IMG_HZ = 30
SAMPLE_SEC = 6
TIME_THRESH = 1

#Chessboard
BOARD_PATTERN = (9, 6)
#in meter
BOARD_SQUARE_SIZE = 0.094
BOARD_WIDTH = 0.695
BOARD_HEIGHT = 0.99
BOARD_DIANGONAL = sqrt(BOARD_WIDTH**2 + BOARD_HEIGHT**2)

#Camera Intrinsic
CAMERA_MATRIX = np.array([[2755.175884, 0.0,      993.842427,],
                 [0.0,     2673.799005,  682.047791,],
                 [0.0, 0.0, 1.0]])
#  DIST_COEF = np.array([-0.747243,  0.266033,  0, 0,  0.214112])
DIST_COEF = np.zeros(5)

#Region of interest in point cloud, only point cloud within this cube will be processed
X_MIN  = 2
X_MAX = 8.6
Y_MIN = -2
Y_MAX = 4
Z_MIN = -2 
Z_MAX = 2

#Throughput and output of the process. No beed to change
PAIR_DF_PATH= "./pair_df.json"
IMG_OUT_DIRECTORY = "./img_calibration_result"
PLANE_CACHE_PATH = "./plane_cache.json"

#Parameters for Lidar detecting chessboard-center
PLANE_DIST_THRESH = 0.05
BEAM_NUMBER = 48

def load_pc_timestamp(pc_directory):
    pc_file_paths = sorted([file_name for file_name in glob.glob(os.path.join(pc_directory, "*.pcd"))])
    pc_timestamps = [float(os.path.basename(file_name[:-4])) for file_name in pc_file_paths]
    return pd.DataFrame({"pc_timestamp": pc_timestamps, "pc_file_path": pc_file_paths})

def get_board_corners(img):
    objp = np.zeros((BOARD_PATTERN[0]*BOARD_PATTERN[1],3), np.float32)
    objp[:,:2] = np.mgrid[0:BOARD_PATTERN[0],0:BOARD_PATTERN[1]].T.reshape(-1,2)
    objp *= BOARD_SQUARE_SIZE
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    ret, corners = cv.findChessboardCorners(gray, (BOARD_PATTERN[0],BOARD_PATTERN[1]), None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        return cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
    else:
        print("Could not found the checkboard!")
        return None

def load_timestamp_fromlog(filepath):
    print("load frame timestamp from", filepath)
    frame_ids = []
    timestamps = []
    with open(filepath) as f:
        for line in f:
            if(len(line.split(" ")) != 9):
                continue
            line = line.strip('\n').split(" ")
            frame_id = int(line[-1])
            timestamp = float(line[5]) / 1e6
            frame_ids.append(frame_id)
            timestamps.append(timestamp)
    # convert frame and timestamp to pandas data frame
    array = np.zeros((len(frame_ids), 2))
    array[:, 0] = frame_ids
    array[:, 1] = timestamps
    frame = pd.DataFrame(array, columns=["image_id", "image_timestamp"]).sort_values(
        "image_timestamp").drop_duplicates()
    frame['image_id'] = frame['image_id'].astype(int)
    return frame

def get_img_center_normal(objp, rvec, tvec):
    r = R.from_rotvec(rvec.squeeze())
    normal = r.as_matrix() * np.matrix([0,0,1], dtype="float").T
    board_center_3d = (objp[0] + objp[-1]) / 2
    board_center_3d = board_center_3d.reshape(1, 3)
    print(CAMERA_MATRIX)
    board_corner_2d = cv.projectPoints(board_center_3d, rvec, tvec, CAMERA_MATRIX, DIST_COEF)
    board_corner_2d = board_corner_2d[0]
    cx = board_corner_2d[0][0][0]
    cy = board_corner_2d[0][0][1]
    center_2d = [cx, cy]
    r = R.from_rotvec(rvec.squeeze()).as_matrix()
    center = np.matmul(r, board_center_3d.T) + tvec
    return center.flatten(), center_2d, np.array(normal).flatten()


def get_img_features(img_directory, handpick_img_directory, ocam=None):
    objp = np.zeros((BOARD_PATTERN[0]*BOARD_PATTERN[1],3), np.float32)
    objp[:,:2] = np.mgrid[0:BOARD_PATTERN[0],0:BOARD_PATTERN[1]].T.reshape(-1,2)
    objp *= BOARD_SQUARE_SIZE
    objpoints = []

    img_log_path = os.path.join(img_directory, "timestamp_vc1.log")
    image_timestamp = load_timestamp_fromlog(img_log_path)
    if handpick_img_directory is None:
        sample_duration = IMG_HZ * SAMPLE_SEC
        sampled_imgs = image_timestamp[image_timestamp.image_id % sample_duration == 1]
        print("Using sampling images!\tSEC:{}\tHZ:{}".format(SAMPLE_SEC, IMG_HZ))
    else:
        img_paths = glob.glob(os.path.join(handpick_img_directory, "*.bmp"))  
        img_ids = []
        for img_path in img_paths:
            file_name = os.path.basename(img_path)
            img_id = file_name.replace("frame_vc1_", "").replace(".bmp", "")
            img_ids.append(int(img_id))
        img_ids = sorted(img_ids)
        sampled_imgs = image_timestamp.iloc[img_ids]
        print("Using hand-picked images!\n {}".format(img_ids))
    target_imgs = {"img_path":[], "img_timestamp":[], "img_normal":[], "img_center_3d":[], "img_center_2d":[], "img_corners":[], "img_quality": []}

    if not os.path.isdir(IMG_OUT_DIRECTORY):
        os.mkdir(IMG_OUT_DIRECTORY)
        print("Generate Directory: {}".format(IMG_OUT_DIRECTORY))


    for img_id in sampled_imgs.image_id.values:
        if img_id < 10:
            img_id = "0{}".format(img_id)
        img_path = os.path.join(img_directory, "frame_vc1_{}.bmp".format(img_id))
        img = cv.imread(img_path)
        #Undistort the image if ocam model is given
        if ocam is not None:
            #ccha97u   ZHANGJUNHAO
            img = ocam.undistort_image(img, img.shape[1], img.shape[0])

        assert img is not None, "\nCan not read img {}".format(img_path)

        img_corners = get_board_corners(img)
        if img_corners is None:
            print("Not able to detect chessboard in {}".format(os.path.basename(img_path)))
            continue
        else:
            print("Detected chessboard in {}".format(os.path.basename(img_path)))
            #  import IPython
            #  IPython.embed()
            img_timestamp = sampled_imgs.loc[int(img_id)].image_timestamp
            points2D = img_corners.squeeze(axis=1)
            points3D = objp
            success, rvec, tvec, inliers = cv2.solvePnPRansac(points3D, points2D, CAMERA_MATRIX, DIST_COEF, flags=cv2.SOLVEPNP_ITERATIVE)
            if not success:
                print("PnP Failed at {}!!!".format(img_path))
                continue
            center_3d, center_2d, normal = get_img_center_normal(objp, rvec, tvec)
            cx, cy = center_2d
            img = cv.circle(img, (cx, cy), radius=10, color=(255, 255, 255), thickness=5)
            cv.drawChessboardCorners(img, BOARD_PATTERN, img_corners, True)
            img_name = os.path.basename(img_path)
            output_path = os.path.join(IMG_OUT_DIRECTORY, img_name)
            cv.imwrite(output_path, img)
            print("image with center written into {}".format(output_path))
            img_quality = inliers.shape[0] /  (BOARD_PATTERN[0] * BOARD_PATTERN[1])

            target_imgs["img_quality"].append(img_quality)
            target_imgs["img_corners"].append(img_corners)
            target_imgs["img_timestamp"].append(img_timestamp)
            target_imgs["img_path"].append(img_path)
            target_imgs["img_normal"].append(normal)
            target_imgs["img_center_3d"].append(center_3d)
            target_imgs["img_center_2d"].append(center_2d)

    return target_imgs


def load_pcd(file_path):
    pcd = o3d.io.read_point_cloud(file_path)
    return np.asarray(pcd.points)    

def get_closeste_point_from_lines(line_a, line_b):
     # define lines A and B by two points
    XA0, UA = line_a.params
    XB0, UB = line_b.params

    # find unit direction vector for line C, which is perpendicular to lines A and B
    UC = cross(UB, UA); UC /= norm(UC)
    # solve the system derived in user2255770's answer from StackExchange: https://math.stackexchange.com/q/1993990
    RHS = XB0 - XA0
    LHS = array([UA, -UB, UC]).T
    t1, t2, t3 = solve(LHS, RHS)
    closest_point = ((XA0+t1*UA) + (XB0+t2*UB)) / 2 
    return closest_point

def render_point_cloud(points, colors, title="Point Cloud"):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title(title)
    ax.set_axis_off()
    ax.set_facecolor((0, 0, 0))
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=colors, s=2, picker=5)
    # Equalize display aspect ratio for all axes
    max_range = (np.array([points[:, 0].max() - points[:, 0].min(), 
        points[:, 1].max() - points[:, 1].min(),
        points[:, 2].max() - points[:, 2].min()]).max() / 2.0)
    mid_x = (points[:, 0].max() + points[:, 0].min()) * 0.5
    mid_y = (points[:, 1].max() + points[:, 1].min()) * 0.5
    mid_z = (points[:, 2].max() + points[:, 2].min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
    return fig, ax


def get_plane_normal(points):
    centroid = points.mean(axis=0)
    centered_points = points - centroid
    u, _, _ = np.linalg.svd(centered_points.T)
    return u[:, -1]

#Further removing points not belong to chessboard plane
def outlier_removal(points, handpick_points):
    handpick_center = handpick_points.mean(axis=0)
    print("Center of handpick points: {}".format(handpick_center))
    dist_to_center = np.linalg.norm(points - handpick_center, axis=1)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[dist_to_center < BOARD_DIANGONAL])
    cl, inliers = pcd.remove_statistical_outlier(nb_neighbors=20,
                                                    std_ratio=2.0) 
    return np.asarray(pcd.select_by_index(inliers).points)

def handpick_plane(points, cache=None):
    conditions = \
        (points[:,0] > X_MIN) &\
        (points[:,0] < X_MAX) &\
        (points[:,1] > Y_MIN) &\
        (points[:,1] < Y_MAX) &\
        (points[:,2] > Z_MIN) &\
        (points[:,2] < Z_MAX)

    print("Before filter {}".format(points.shape))
    points = points[conditions]
    print("After filter {}".format(points.shape))
    print(points.shape)

    # Color map for the points
    cmap = matplotlib.cm.get_cmap('hsv')
    colors = cmap(points[:, -1] / np.max(points[:, -1]))

    if cache is None:
        fig, ax = render_point_cloud(points, colors, "Hand Pick point cloud")
    # Setup matplotlib GUI
    # Pick points
    lines = []
    picked, corners = [], []
    def onpick(event):
        ind = event.ind[0]
        x, y, z = event.artist._offsets3d

        # Ignore if same point selected again
        if picked and (x[ind] == picked[-1][0] and y[ind] == picked[-1][1] and z[ind] == picked[-1][2]):
            return
        
        # Display picked point
        picked.append((x[ind], y[ind], z[ind]))
        corners.append((x[ind], y[ind], z[ind]))
        print("Picked point: {}".format(corners[-1]))

        if len(picked) > 1:
            # Draw the line
            temp = np.array(picked)
            line, = ax.plot(temp[:, 0], temp[:, 1], temp[:, 2])
            lines.append(line)
            ax.figure.canvas.draw_idle()

            # Reset list for future pick events
            del picked[0]

    # Remove points when mis-clicicked
    def onkeyboard(event):
        if event.key == 'k':
            while(len(corners)):
                corners.pop()
            while(len(picked)):
                picked.pop()
            while(len(lines)):
                line = lines.pop()
                line.remove()
            ax.figure.canvas.draw_idle()
            print("Remove all selected points")

    if cache is None:
    # Display GUI
        fig.canvas.mpl_connect('pick_event', onpick)
        fig.canvas.mpl_connect('key_press_event', onkeyboard)
        plt.show()
    else:
        corners = cache

    if len(corners) >= 3:
        print("Fitting the plane")
        pc_normal = get_plane_normal(np.array(corners))
        corner_points = np.array(corners)
        d = -np.matmul(corner_points, np.matrix(pc_normal).T).mean()
        plane_dist = np.array(np.matmul(points, np.matrix(pc_normal).T))
        plane_dist = np.abs(plane_dist + d)
        valid_counts = (plane_dist < PLANE_DIST_THRESH).sum()
        print("plane_dist mean {}. Valid Points {}".format(plane_dist.mean(), valid_counts))
        condition = (plane_dist < PLANE_DIST_THRESH).squeeze()
        #Further remove points of tripod and noisy points
        plane_points = outlier_removal(points[condition], corner_points)
        print("Plane Filtering. Before {} After {}".format(points.shape[0], plane_points.shape[0])) 
        return plane_points, pc_normal, corner_points
    else:
        print("Pick at lease 3 points to fit the plane!")
        return points, None, corners
    # Save corner points

def detect_pc_center(points):
    ring = np.arctan2(points[:, 2], np.linalg.norm(points[:, [0,1]]))
    max_ring = ring.max()
    min_ring = ring.min()
    beam_range = (max_ring - min_ring) / BEAM_NUMBER
    print(beam_range)
    ring_count = ((ring - min_ring) / beam_range).astype(int)
    ring = ring - ring.mean()
    print(ring_count)
    df = pd.DataFrame(ring)
    print(df.describe())

    #  cmap = matplotlib.cm.get_cmap('hsv')
    cmap = matplotlib.colors.ListedColormap(['k','b','y','g','r'])
    #  colors = cmap(ring_count%5)
    colors = np.zeros(points.shape[0])
    df = pd.DataFrame(points, columns = ["x", "y", "z"])
    df["ring"] = ring_count

    min_point_ids = df.groupby("ring").y.idxmin().values
    min_points = points[min_point_ids][:, :3]
    max_point_ids = df.groupby("ring").y.idxmax().values
    max_points = points[max_point_ids][:, :3]
    min_line_1, inliers = ransac(min_points, LineModelND, min_samples=2,
                               residual_threshold=0.02, max_trials=1000)
    colors[min_point_ids[inliers]] = 1
    min_main_points = min_points[inliers]
    rest_point_ids = min_point_ids[~inliers]
    rest_points = min_points[~inliers]
    min_line_2, inliers = ransac(rest_points, LineModelND, min_samples=2,
                               residual_threshold=0.02, max_trials=1000)
    colors[rest_point_ids[inliers]] = 2
    min_sub_poins = rest_points[inliers]

    #Decide which edge is up edge and down edge
    min_main_z = min_main_points[:, 2].mean()
    min_rest_z = min_sub_poins[:, 2].mean()
    if(min_main_z > min_rest_z):
        up_min_edge = min_line_1
        down_min_edge = min_line_2
    else:
        up_min_edge = min_line_2
        down_min_edge = min_line_1


    #  colors[min_point_ids[~inliers]] = 2
    max_line_1, inliers = ransac(max_points, LineModelND, min_samples=2,
                               residual_threshold=0.02, max_trials=1000)
    colors[max_point_ids[inliers]] = 3
    max_main_points = max_points[inliers]
    #  colors[max_point_ids[~inliers]] = 4

    rest_point_ids = max_point_ids[~inliers]
    rest_points = max_points[~inliers]
    max_line_2, inliers = ransac(rest_points, LineModelND, min_samples=2,
                               residual_threshold=0.02, max_trials=1000)
    colors[rest_point_ids[inliers]] = 4
    max_sub_points = rest_points[inliers]

    #Decide which edge is up edge and down edge
    max_main_z = max_main_points[:, 2].mean()
    max_rest_z = max_sub_points[:, 2].mean()
    if(max_main_z > max_rest_z):
        up_max_edge = max_line_1
        down_max_edge = max_line_2
    else:
        up_max_edge = max_line_2
        down_max_edge = max_line_1

    colors = cmap(colors.astype(int))
   

    corner_1 = get_closeste_point_from_lines(up_max_edge, up_min_edge)
    corner_2 = get_closeste_point_from_lines(down_max_edge, down_min_edge)
    corner_3 = get_closeste_point_from_lines(up_min_edge, down_min_edge)
    corner_4 = get_closeste_point_from_lines(up_max_edge, down_max_edge)
    board_corners = np.array([corner_1, corner_2, corner_3, corner_4])
    estimated_center = board_corners.mean(axis=0)
    board_corners_list = board_corners.tolist()
    board_corners_list.append(estimated_center.tolist())
    board_corners = np.array(board_corners_list)

    to_save = False
    
    def onkeyboard(event):
        nonlocal to_save
        if event.key == 'x':
            to_save = True
            print("save the result")

    fig, ax = render_point_cloud(points, colors)
    fig.canvas.mpl_connect("key_press_event", onkeyboard)
    ax.scatter(board_corners[:,0], board_corners[:,1], board_corners[:, 2], c='w', s=50)
    plt.show()
    return to_save, estimated_center

def feature_to_string(img_quality, img_normal, img_center_3d, img_center_2d, pc_center_quality, pc_normal, pc_center):
    inx, iny, inz = img_normal
    icx, icy, icz = img_center_3d
    icu, icv = img_center_2d
    pnx, pny, pnz = pc_normal
    pcx, pcy, pcz = pc_center
    s = "{}, {}, {}, {}, {}, {}, {}, {}, {}".format(img_quality, inx, iny, inz, icx, icy, icz, icu, icv)
    s = s + ", {}, {}, {}, {}, {}, {}, {}\n".format(pc_center_quality, pnx, pny, pnz, pcx, pcy, pcz)
    return s


#Input is a single record in pair_df
def calibrate(pair_recrod, feature_file, failed_pointclouds, plane_cache):
    pc_path = pair_recrod.pc_file_path
    points = load_pcd(pc_path)
    pc_name = os.path.basename(pc_path)
    cache = plane_cache.get(pc_name, None)
    if cache is not None:
        print("Using cache for {}".format(pc_name))

    plane_points, pc_normal, handpick_points = handpick_plane(points, cache)

    if type(handpick_points) is np.ndarray:
        handpick_points = handpick_points.tolist()
    if pc_normal is None:
        "Give up this pair of data"
        return
    if pc_name not in plane_cache:
        print("Written cache into {}".format(pc_name))
    plane_cache[pc_name] = handpick_points
    
    success, pc_center = detect_pc_center(plane_points)
    if success:
        pc_center_quality = 1
    else:
        pc_center_quality = 0
        failed_pointclouds.append(record.pc_file_path)
        print("Corner Detection Failed at {}".format(failed_pointclouds[-1]))

    img_center_3d = pair_recrod.img_center_3d
    img_normal = pair_recrod.img_normal
    img_center_3d = pair_recrod.img_center_3d
    img_center_2d = pair_recrod.img_center_2d
    img_quality = pair_recrod.img_quality
    featrue_string = feature_to_string(img_quality, img_normal, img_center_3d, img_center_2d, pc_center_quality, pc_normal, pc_center)
    with open(feature_file, 'a') as w:
        w.write(featrue_string)
        print("Feature written into {}\n {}".format(feature_file, featrue_string))

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Lidar-Camera extrinsic calibration tool")
    parser.add_argument("-i", "--image_feature", action="store_true", help="Extract image features for calibration. If not specified, point cloud extraction will be launched")
    parser.add_argument("-f", "--feature_file", help="Output file path of the features", default=None)
    parser.add_argument("-m", "--handpick_img_directory", default=None, help="Directory of hand-picked images. Sampling stragety  will be used if not specified")
    parser.add_argument("-n", "--no_cache", action="store_true" ,help="Not using the cache of plane-picking in point cloud")
    parser.add_argument("-o", "--ocam_intrinsic_path", help="File path of the ocam intrinsic", default=None)
    args = parser.parse_args()
    assert args.feature_file != None or args.image_feature == True, "Please specify output path of features or -i to start image feature extraction mode"

    plane_cache = {}
    if os.path.isfile(PLANE_CACHE_PATH):
        plane_cache = json.load(open(PLANE_CACHE_PATH, 'r'))

    
    if args.image_feature:
        if args.ocam_intrinsic_path is None:
            ocam = None
        else:
            ocam = OcamCamera(args.ocam_intrinsic_path)
            CAMERA_MATRIX = ocam.get_virtual_K()

        img_features = get_img_features(IMG_DIRECTORY, args.handpick_img_directory, ocam)
        pc_timestamps = load_pc_timestamp(PC_DIRECTORY)
        img_df = pd.DataFrame(img_features)
        pair_df = pd.merge_asof(img_df, pc_timestamps, left_on="img_timestamp", right_on="pc_timestamp", direction="nearest", tolerance=TIME_THRESH)
        pair_df = pair_df.dropna()
        pair_df["diff"] = (pair_df.img_timestamp - pair_df.pc_timestamp).abs().values
        print("Written pair df into {}".format(PAIR_DF_PATH))
        pair_df.to_json(PAIR_DF_PATH)
    else:
        print("Features will be written into {}".format(args.feature_file))
        pair_df = pd.read_json(PAIR_DF_PATH)
        failed_pointclouds = []
        with open(args.feature_file, 'w') as w:
            w.write("img_quality, inx, iny, inz, icx, icy, icz, icu, icv, pc_center_quality, pnx, pny, pnz, pcx, pcy, pcz\n")
        for record in pair_df.iloc:
            calibrate(record, args.feature_file, failed_pointclouds, plane_cache)

        with open("failed_pointclouds", 'w') as w:
            for failed_pointcloud in failed_pointclouds:
                w.write(failed_pointcloud + "\n")
        json.dump(plane_cache, open(PLANE_CACHE_PATH, 'w'))
        print("Hand-picked points cache written into {}".format(PLANE_CACHE_PATH))
