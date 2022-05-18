#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2 as cv
import open3d as o3d
from feature_extractor import IMG_DIRECTORY, PC_DIRECTORY, load_pcd, load_pc_timestamp, load_timestamp_fromlog
from ocamcamera import OcamCamera
import argparse
import numpy as np
import pandas as pd
import os
import matplotlib
import glob

IMAGE_WIDTH = 1920
IMAGE_HEIGHT = 1080
FOREGROUND_THRESH = 6

T_lc = np.array([[ 0.02584355, -0.99955999, -0.01455811, -0.05892653],
                  [-0.06588959,  0.0128281,  -0.99774446, -0.38580909],
                  [ 0.99749219,  0.02674449, -0.06552907, -0.45402385],
                  [ 0.,          0.,          0.,          1.        ]])


CAMERA_MATRIX = np.array([[2755.175884, 0.0,      993.842427,],
                 [0.0,     2673.799005,  682.047791,],
                 [0.0, 0.0, 1.0]])

def project_point_cloud(points3D):
    points3D_h = np.ones((4, points3D.shape[0]))
    points3D_h[:3, :] = points3D.T 
    
    points_C = (T_lc @ points3D_h)[:3, :]

    #Project points into the UV-Plane
    points_uv = (CAMERA_MATRIX @ points_C)[:2]
    points_z = points_C[2]
    points_uv = (points_uv / points_z).round().astype(int)

    mask = (points_uv[0, :] >=0) &\
        (points_uv[0, :] < IMAGE_WIDTH) &\
        (points_uv[1, :] >=0) &\
        (points_uv[1, :] < IMAGE_HEIGHT)
    points_uv_valid = points_uv[:, mask]
    points_z_valid = points_z[mask]
    
    return points_uv_valid.T, points_z_valid


def visualize(img_path, pcd_path, ocam=None):
    points3D = load_pcd(pcd_path)
    img = cv.imread(img_path)
    if ocam is not None:
        img = ocam.undistort_image(img, IMAGE_WIDTH, IMAGE_HEIGHT)
    assert img is not None, "Unable to read image from {}".format(img_path)
   
    points_uv_valid, points_z_valid = project_point_cloud(points3D)

    #Filter out background points
    foreground_mask = points_z_valid < FOREGROUND_THRESH
    points_uv_valid= points_uv_valid[foreground_mask]
    points_z_valid= points_z_valid[foreground_mask]
    if(points_uv_valid.shape[0] == 0):
        print("Image {} failed to be visualized because no points are in foreground!".format(img_path))
        return img
    
    max_z = points_z_valid.max()
    cmap = matplotlib.cm.get_cmap('jet')
    colors = cmap(points_z_valid / max_z) * 255
    #  import IPython
    #  IPython.embed()
    for i in range(len(points_uv_valid)):
        cv.circle(img, tuple(points_uv_valid[i]), 2, tuple(colors[i]), -1)
    return img

    # Project to 2D and filter points within image boundaries


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualizaer for point-cloud and image")
    parser.add_argument("-o", "--ocam_intrinsic_path", help="File path of the ocam intrinsic", default=None)
    parser.add_argument("-l", "--limit", help="How many visualization image to be saved", default=10)
    parser.add_argument("-m", "--handpick_img_directory", default=None, help="Directory of hand-picked images. Sampling stragety  will be used if not specified")
    parser.add_argument("--output_dir", help="Directory to save the visualization images", default="./visualization_result")
    args = parser.parse_args()
    TIME_THRESH = 0.1

    if args.ocam_intrinsic_path is None:
        ocam = None
    else:
        ocam = OcamCamera(args.ocam_intrinsic_path)
        CAMERA_MATRIX = ocam.get_virtual_K()
    if not os.path.isdir(args.output_dir):
        os.mkdir(args.output_dir)
    

    pc_timestamp = load_pc_timestamp(PC_DIRECTORY)
    img_timestamp = load_timestamp_fromlog(os.path.join(IMG_DIRECTORY, "timestamp_vc1.log"))

    if args.handpick_img_directory is not None:
        img_paths = glob.glob(os.path.join(args.handpick_img_directory, "*.bmp"))  
        img_ids = []
        for img_path in img_paths:
            file_name = os.path.basename(img_path)
            img_id = file_name.replace("frame_vc1_", "").replace(".bmp", "")
            img_ids.append(int(img_id))
        img_ids = sorted(img_ids)
        img_timestamp = img_timestamp.iloc[img_ids]
        print("Using hand-picked images!\n {}".format(img_ids))

    pair_df = pd.merge_asof(img_timestamp, pc_timestamp, left_on="image_timestamp", right_on="pc_timestamp", direction="nearest", tolerance=TIME_THRESH).dropna()
    
    for i, pair_record in enumerate(pair_df.iloc):
        if i >= args.limit:
            break
        pcd_path = pair_record.pc_file_path
        img_id = pair_record.image_id
        if img_id < 10:
            img_id = "0{}".format(img_id)
        img_path = os.path.join(IMG_DIRECTORY, "frame_vc1_{}.bmp".format(img_id))

        project_img = visualize(img_path, pcd_path, ocam)
        output_path = os.path.join(args.output_dir, os.path.basename(img_path))
        cv.imwrite(output_path, project_img)
        print("Visualization result  written into {}".format(output_path))
        
