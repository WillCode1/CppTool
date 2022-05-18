# Offline Lidar Calibration with Chessboard

This tool estimate the transformation matrix **from Lidar frame to Camera frame**.  
RoboSense M1 Lidar has been successfully calibrated by this tool. The following instruction will be using RoboSense-M1 as example, but this tool can be applied to different Lidar as well.

## 1. Requirements

1. A Calibration Chessboard
2. A Tripod for holding the board
3. RoboSense Lidar Driver --->Please refer to **lidar data recording manual**



## 2. Setup for Chess Board

### 2.1 Hold the chessboard by tripod in 45-degree tilted manner. 
![](./fig/good_tilt.jpg)
- **Do not hold chessboard by hands**.  
Hand-holding cause instability and unsynchronized data pair due to the small time difference between Lidar and camera. 

![](./fig/hand_hold_bad.jpg)


- **Do not hold chessboard in flat manner**.  
Edges of a flat chessboard can not be detected due to Lidar property and algorithm design

![](./fig/bad_flat.jpg)

### 2.2 Rotate the tripod and chessboard in multiple positions during recording

## 3. Data Recording of Lidar and Camera
Please refer to the [data recording instruction](./lidar_recording.md)

## 4. Data Preprocessing

### 4.1 Lidar
Transform the point-cloud in rosbag into pcd files.  
The following is the ROS coomand. Please make sure **ROS** and **pcl_ros** packages are installed before using it.

    $ rosrun pcl_ros bag_to_pcd ./my_lidar.bag /rslidar_topic_name ./pcd_directory 
### 4.2 Camera
No preprocess is needed for images. Just make sure images and the time-log file is properly stored.

## 5. Calibration Process Overview
In the process, the **Feature Extractor** firstly extract image features, then extract point cloud features. Two different features will be fused and written into a CSV file.  
The **Extrinsic Estimator** takes the CSV file and compute the extrinsic.
![](./fig/overview.png)


## 6. Python Dependencies of Calibration Tool
1. pandas
2. matplotlib
3. skimage
4. open3d
5. opencv
6. scipy
7. numpy

## 7, Getting Started
The sample_data_2021_07_16 in repo support hand-picked input mode only. You can try it by the following command: 

    #Extract image features
    $ python3 feature_extractor.py -i -m ./sample_data_2021_07_16/image/
    #Extract point cloud features
    $ python3 feature_extractor.py -f ./feature.csv -m ./sample_data_2021_07_16/image/
    #Estimate the Extrinsic from lidar frame to camera frame
    $ python3 extrinsic_estimator.py ./feature.csv

For Ocam images, please modify the IMG_DIRECTORY and PC_DIRECTORY variable in **feature_extractor.py**.  
And then run the following command

    #Extract image features
    $ python3 feature_extractor.py -i -m ./sample_data_2021_08_23_ocam/image/ -o ./sample_data_2021_08_23_ocam/ocam_intrinsics.txt
    #Extract point cloud features
    $ python3 feature_extractor.py -f ./feature.csv -m ../sample_data_2021_08_23_ocam/image/ -o ./sample_data_2021_08_23_ocam/ocam_intrinsics.txt
    #Estimate the Extrinsic from lidar frame to camera frame
    $ python3 extrinsic_estimator.py ./feature.csv -o ./sample_data_2021_08_23_ocam/ocam_intrinsics.txt


## 7. Instruction of Feature Extractor
#### The sample_data_2021_07_16 only support hand-picked input mode! Please try the sample with **-m** argument

### 7.1 Config  
Before using the feature extractor. Properly change the config in the Python script.  
You will need to change the input directory, board parameter, and camera intrinsic.


    #Input directory
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
    DIST_COEF = np.array([-0.747243,  0.266033,  0, 0,  0.214112])

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

### 7.2 Image Feature
Firstly, use **-i** command to extract image features. (You have to set the correct input directory beforehand)  
To check if the board-center is extracted correctly, you can see the visualization result in **./img_calibration_result**.  
The result is correct in most of the case.

![](./fig/camera_feature.png)

    $ python3 feature_extractor.py -i
    load frame timestamp from /home/ccha97u/data/user/lidar_calibration/0716/image_record/2016-2-12-0-40-4_v1/raw/nm_000005_00000100_08/timestamp_vc1.log
    Using sampling images!  SEC:6   HZ:30
    Could not found the checkboard!
    Not able to detect chessboard in frame_vc1_01.bmp
    Could not found the checkboard!
    Not able to detect chessboard in frame_vc1_541.bmp
    Detected chessboard in frame_vc1_721.bmp
    image with center written into ./img_calibration_result/frame_vc1_721.bmp
    Detected chessboard in frame_vc1_901.bmp
    image with center written into ./img_calibration_result/frame_vc1_901.bmp
    ......
    Written pair df into ./pair_df.json

Some images will not be able to extracted, it's normal.

### 7.3 Point-Cloud Feature
The **-f** argument specifies where the features will be stored.  
The tool automatically find the corresponding PCD file and extract the point cloud feature one by one

    $ python3 feature_extractor.py -f ./features.csv
- **Manually pick more than 3 points of the chessboard. Any point of the chessboard can be selected.**

In the following example, green points in the center represent the chessboard. Pick some points of chessboard. More than 3 points should be selected. After selection is done, close the window.


![](./fig/hand_pick.png)  



Then, the visualization of corner detection will pop up. **You have to judge whether the detection result is correct**. A correct result will have four dots composing a rectangular plane in 3D-space and one dot representing the center.  
In contrast, the failed case is unable to form a rectangle by four dots.  

- **If the result is correct. Press 'x' to save the result and close the window** 
- **If the result is wrong, directly close the window**

#### **Good Detection**
![](./fig/corner_good.png)  

#### **Failed Detection**
![](./fig/corner_fail.png)  




### 7.4 Hand-picked input
Manual selecting input images is also supported. Copy the images into a given directory(*Do not change the image names*). Use the **-m** argument to extract the handpick images

    # Image Extraction
    $ python3 feature_extractor.py -i -m ./input_handpick_imgs/

    #Point Cloud Extraction
    $ python3 feature_extractor.py f ./features.csv -m ./input_handpick_imgs/

### 7.5 Ocam Images
If the images is using Ocam model. Simply using **-o** argument followed by the path of the ocam intrinsic file.  
Please refer to [https://github.com/matsuren/ocamcalib_undistort](https://github.com/matsuren/ocamcalib_undistort) for more details about Ocam model.

    #Extract image features
    $ python3 feature_extractor.py -i -m ./sample_data_2021_08_23_ocam/image/ -o ./sample_data_2021_08_23_ocam/ocam_intrinsics.txt
    #Extract point cloud features
    $ python3 feature_extractor.py -f ./feature.csv -m ../sample_data_2021_08_23_ocam/image/ -o ./sample_data_2021_08_23_ocam/ocam_intrinsics.txt
    #Estimate the Extrinsic from lidar frame to camera frame
    $ python3 extrinsic_estimator.py ./feature.csv -o ./sample_data_2021_08_23_ocam/ocam_intrinsics.txt

## 8. Instruction of Extrinsic Estimator
Once the **feature.csv** file is written. Extrinsic can be estimated


    $ python3 extrinsic_estimator.py  

    ===== 3D-2D correspondence =====
    Euler angles (RPY): [-2.94627765 -1.54670799 -1.78182928]
    Euler angles (YPR): -1.7818292783357095 -1.546707990705663 -2.9462776518211196|
    Translation in meter (XYZ): [-0.07753906 -0.43324357 -0.1627684 ]
    Visualization Format
    -0.07753906427483438 -0.43324357063170743 -0.16276840199065915 -1.7818292783357095 -1.546707990705663 -2.9462776518211196
    Transformation Matrix
    [[-0.0050453  -0.99986468 -0.01565769 -0.07753906]
    [-0.02355166  0.01577236 -0.9995982  -0.43324357]
    [ 0.99970989 -0.00467451 -0.02362805 -0.1627684 ]
    [ 0.          0.          0.          1.        ]]

    The mean difference(meter) between lidar-center and camera-center in camera-frame
    [-0.00179761 -0.00693599  0.10083841]


## 9. Visualization
You can visualize the result by **visualizer.py**

    $ python3 visualizer.py -m sample_data_2021_08_23_ocam/image -o sample_data_2021_08_23_ocam/ocam_intrinsics.txt
    Loading virtual fx:1173.913043 and fy:1173.913043
    load frame timestamp from /home/ccha97u/data/user/lidar_calibration/lidar_calibration_0823/image_record/0/timestamp_vc1.log
    Using hand-picked images!
    [1412, 1779, 2129, 2541, 2889, 3250, 3644, 3962, 4467, 4889, 5207, 5542, 5885, 6204, 6466, 6743]
    Visualization result  written into ./visualization_result/frame_vc1_1412.bmp
    Visualization result  written into ./visualization_result/frame_vc1_1779.bmp
    Visualization result  written into ./visualization_result/frame_vc1_2129.bmp
    Visualization result  written into ./visualization_result/frame_vc1_2541.bmp
    Visualization result  written into ./visualization_result/frame_vc1_2889.bmp
    Visualization result  written into ./visualization_result/frame_vc1_3250.bmp
    Visualization result  written into ./visualization_result/frame_vc1_3644.bmp
    Visualization result  written into ./visualization_result/frame_vc1_3962.bmp
    Visualization result  written into ./visualization_result/frame_vc1_4467.bmp
    Visualization result  written into ./visualization_result/frame_vc1_4889.bmp

**visualizer.py** also supports **-o** argument for Ocam images

![](visualization_result.bmp)