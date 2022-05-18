#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation as R
from feature_extractor import CAMERA_MATRIX, DIST_COEF
import cv2
import argparse
from ocamcamera import OcamCamera

def estimate_rotation(Nc, Nl):
    R_lc = np.linalg.inv(Nl*Nl.T) * (Nl*Nc.T)
    return R_lc.T

def get_Nc_Nl(df):
    Nc = df[["inx", "iny", "inz"]].values
    Nl = df[["pnx", "pny", "pnz"]].values
    Nc = np.matrix(Nc)
    Nl = np.matrix(Nl)
    for nl in Nl:
        if nl[0, 0] > 0:
            nl *= -1
    for nc in Nc:
        if nc[0, 2] > 0:
            nc *= -1
    return Nc, Nl

def get_Oc_Ol(df):
    #remove bad quality lidar center
    #  better_df = df[df.pc_center_quality == 1]
    better_df = df

    Oc = better_df[["icx", "icy", "icz"]].values
    Ol = better_df[["pcx", "pcy", "pcz"]].values
    return Oc.T, Ol.T


if __name__ == "__main__":

    np.set_printoptions(suppress=True) 
            
    parser = argparse.ArgumentParser(description="Lidar-Camera extrinsic solver")
    parser.add_argument("feature_file", help="the path of the featur file")
    parser.add_argument("-q", "--image_quality", help="Threshold for img_quality", type=float, default=0)
    parser.add_argument("-p", "--pointcloud_strict", action="store_true", help="To discard bad pointcloud center or not")
    parser.add_argument("-3", "--show_3d3d", action="store_true", help="Whether to show 3D3D extrinsic")
    parser.add_argument("-o", "--ocam_intrinsic_path", help="File path of the ocam intrinsic", default=None)
    args = parser.parse_args()

    if args.ocam_intrinsic_path is None:
        ocam = None
    else:
        ocam = OcamCamera(args.ocam_intrinsic_path)
        CAMERA_MATRIX = ocam.get_virtual_K()

    df = pd.read_csv(args.feature_file, skipinitialspace=True)
    df = df[df.img_quality >= args.image_quality]
    Nc, Nl = get_Nc_Nl(df)
    if args.pointcloud_strict:
        df = df[df.pc_center_quality == 1]
    #  Nc, Nl = get_Nc_Nl(df)
    R_cl, rmse  = R.align_vectors(Nc, Nl)
    r_cl = R_cl.as_matrix()

    Oc, Ol = get_Oc_Ol(df)

    T_cl_total =  (Oc - np.matmul(r_cl, Ol))
    T_cl = T_cl_total.mean(axis=1)


    T_3d3d = np.identity(4)
    T_3d3d[:3, :3] = R_cl.as_matrix()
    T_3d3d[:3, 3] = T_cl
    #Transform rotation into Euler angles 
    euler_rpy = R_cl.as_euler("xyz")
    if args.show_3d3d:
        print("\n===== Normal and 3D-center correspondence =====")
        print("WARNING!!!!! This method is not able to give a good estimation currently")
        print("Euler angles (RPY): {}\t".format(euler_rpy))
        print("Euler angles (YPR): {} {} {}|\t".format(euler_rpy[-1], euler_rpy[1], euler_rpy[0]))
        print("Translation in meter (XYZ): {}".format(T_cl))
        print("Visualization Format\n{} {} {} {} {} {}".format(T_cl[0], T_cl[1], T_cl[2], euler_rpy[-1], euler_rpy[1], euler_rpy[0]))
        print("Transformation Matrix\n{}".format(T_3d3d))

    
    print("\n===== 3D-2D correspondence =====")
    points2D = df[["icu", "icv"]].values
    points3D = Ol.T
    success, rvec, tvec, inliers = cv2.solvePnPRansac(points3D, points2D, CAMERA_MATRIX, DIST_COEF, flags=cv2.SOLVEPNP_ITERATIVE)
    rvec = rvec.flatten()
    tvec = tvec.flatten()
    r_2d = R.from_rotvec(rvec.flatten())
    euler_2d = r_2d.as_euler("xyz")

    T_2d3d = np.identity(4)
    T_2d3d[:3, :3] = r_2d.as_matrix()
    T_2d3d[:3, 3] = tvec

    print("Euler angles (RPY): {}\t".format(euler_2d))
    print("Euler angles (YPR): {} {} {}|\t".format(euler_2d[-1], euler_2d[1], euler_2d[0]))
    print("Translation in meter (XYZ): {}".format(tvec.flatten()))
    print("Visualization Format\n{} {} {} {} {} {}".format(tvec[0], tvec[1], tvec[2], euler_2d[-1], euler_2d[1], euler_2d[0]))
    print("Transformation Matrix\n{}".format(T_2d3d))


    #Estimate the difference between lidar-3d points and camera-3d points
    #Transform lidar-center into camera frame
    Olc = np.matmul(R.from_rotvec(rvec).as_matrix(), Ol) + tvec.reshape(3,1)
    O_diff = Olc - Oc
    print("\nThe mean difference(meter) between lidar-center and camera-center in camera-frame\n {}".format(O_diff.mean(axis=1)))

