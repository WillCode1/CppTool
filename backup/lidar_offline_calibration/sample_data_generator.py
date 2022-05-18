#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pandas as pd
from feature_extractor import load_pc_timestamp, load_timestamp_fromlog
import argparse
import os
import glob
from subprocess import call

IMG_DIRECTORY = "/home/ccha97u/data/user/lidar_calibration/lidar_calibration_0823/image_record/0"
PC_DIRECTORY = "/home/ccha97u/data/user/lidar_calibration/lidar_calibration_0823/pcd"
OUTPUT_PATH = "/home/ccha97u/Documents/calibration/lidar_offline_calibration/sample_data_2021_08_23_ocam"
HAND_PICKED_DIRECTORY = "/home/ccha97u/Documents/calibration/lidar_offline_calibration/picked_imgs/"
TIME_THRESH = 0.5

if __name__  == "__main__":
    if not os.path.isdir(OUTPUT_PATH):
        os.mkdir(OUTPUT_PATH)
    if not os.path.isdir(os.path.join(OUTPUT_PATH, "image")):
        os.mkdir(os.path.join(OUTPUT_PATH, "image"))
    if not os.path.isdir(os.path.join(OUTPUT_PATH, "pcd")):
        os.mkdir(os.path.join(OUTPUT_PATH, "pcd"))
    picked_img_paths = glob.glob(os.path.join(HAND_PICKED_DIRECTORY, "*.bmp"))
    picked_img_ids = [int(os.path.basename(picked_img_path).replace("frame_vc1_", "").replace(".bmp", "")) for picked_img_path in picked_img_paths]
    img_timestamp = load_timestamp_fromlog(os.path.join(IMG_DIRECTORY, "timestamp_vc1.log"))
    pc_timestamp = load_pc_timestamp(PC_DIRECTORY)
    pair_df = pd.merge_asof(img_timestamp, pc_timestamp, left_on="image_timestamp", right_on="pc_timestamp", direction="nearest", tolerance=TIME_THRESH).dropna()
    for picked_img_id in picked_img_ids:
        pair_record = pair_df.loc[picked_img_id]
        img_id = pair_record.image_id
        if img_id < 10:
            img_id = "0{}".format(img_id)
        img_path = os.path.join(IMG_DIRECTORY, "frame_vc1_{}.bmp".format(img_id))
        pc_path = pair_record.pc_file_path
        target_img_path = os.path.join(OUTPUT_PATH, "image", "frame_vc1_{}.bmp".format(img_id))
        target_pc_path = os.path.join(OUTPUT_PATH, "pcd", os.path.basename(pc_path))
        CMD = ["cp", img_path, target_img_path]
        print(" ".join(CMD))
        call(CMD)
        print(" ".join(CMD))
        CMD = ["cp", pc_path, target_pc_path]
        call(CMD)





