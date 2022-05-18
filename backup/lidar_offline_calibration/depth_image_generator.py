#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rosbag
import pandas as pd
import numpy as np
import ros_numpy
import cv2
import matplotlib.pyplot as plt



#Extrinsic from lidar frame to camera frame
T_lc = np.array([[-0.00623602, -0.99987477, -0.0145449,  -0.06984721],
                 [-0.02383271,  0.01468966, -0.99960803, -0.43222822],
                 [ 0.99969651, -0.00588693, -0.02392133, -0.17066517],
                 [ 0.,          0.,          0.,          1.        ]])
#Intrinsic of the camera
K = np.array([[2755.175884, 0.0,      993.842427,],
     [0.0,     2673.799005,  682.047791,],
     [0.0, 0.0, 1.0]])

IMAGE_WIDTH = 1920
IMAGE_HEIGHT = 1208

def RobosenseToNumpy(msg):
        points_array = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
        print(points_array.shape)
        import IPython
        IPython.embed()
        if(len(points_array.shape)) == 2:
            df = pd.DataFrame(points_array[0])
            for points in points_array:
                df = pd.concat([df, pd.DataFrame(points)])
        else:
            df = pd.DataFrame(points_array)
        print(df.shape)
        return df.dropna().values



if __name__ == "__main__":
    #  bag_path = "/media/zhangjunhao/DataRecord1T/2021-08-17-15-29-22.bag"
    bag_path = "/home/ccha97u/data/user/lidar_calibration/lidar_calibration_0823/lidar0823_2021-08-23-16-18-36_correct.bag"
    target_topics = "/rslidar_points"

    #Read the rosbag
    bag = rosbag.Bag(bag_path)
    for topic, msg, recv_t in bag.read_messages([target_topics]):
        #Extract point cloud
        points_L = RobosenseToNumpy(msg)

        #Firstly transform points into homogeneous coordinate
        points_L[:, 3] = 1

        #Then, transform point cloud from Lidar frame into Camera Frame
        points_C = (T_lc @ points_L.T)[:3, :]

        #Project points into the UV-Plane
        points_uv = (K @ points_C)[:2]
        points_z = points_C[2]
        points_uv = (points_uv / points_z).round().astype(int)

        #Filter out points outside of UV boundary
        mask = (points_uv[0, :] >=0) &\
            (points_uv[0, :] < IMAGE_WIDTH) &\
            (points_uv[1, :] >=0) &\
            (points_uv[1, :] < IMAGE_HEIGHT)
        points_uv_valid = points_uv[:, mask]
        points_z_valid = points_z[mask]

        #Generate depth_img
        depth_img = np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH)) 
        depth_img[points_uv_valid[1], points_uv_valid[0]] = points_z_valid

        #Normalize depth into the range of 0 ~ 255
        depth_img = depth_img / depth_img.max() * 255
        depth_img = depth_img.astype(np.uint8)


        #  break
    #Make closer points brighter
    #  depth_img[depth_img > 0] = 255 - depth_img[depth_img > 0]
#
    #  cv2.imshow('Normalized depth img', depth_img)
    #  cv2.waitKey(0)
    #  cv2.destroyAllWindows()

            
                

