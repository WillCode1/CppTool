# Mono Camera Intrinsics Calibration Tool

[toc]

## Build

All third party are used ***3rdparty*** repo, please check the right path in `CMakeLists.txt`, then start building:

`mkdir build`

`cd build`

`cmake ..`

`make`

## Usage

### Calibration Tool

#### Config

1. Change calibration parameters at  `./config/config.cfg`

2. The first 6 parameters ***must be*** filled in:

   - `calib_type` :  pinhole, fisheye, ocam
   - `image_path` : only need set the path, it will detect all image format
   - `image_size` : image width and height
   - `board_size` : size of chessboard pattern (e.g. 9*6)
   - `board_unit_length` : real length of unit pattern (e.g 50 mm)
   - `show_undistort` : if show the undistorted images after calibration  (1 --> enable, 0 --> disable)

3. There are two calibration parameters of fisheye model:

   - `pinhole_fix_p` : set tangential distortion p1, p2 to 0.0 (1 --> enable, 0 --> disable)

   - `pinhole_fix_k3` : set radial distortion k3 to 0.0 (1 --> enable, 0 --> disable)

     Pinhole calibration estimates 5 dist_coeffs by default (k1, k2, p1, p2, k3)

4. If use **ocam** model, more undistortion option are provided, it will be introduced in the next section.

#### Run

***Step 1:***

`cd ./bin/`

`./mono_intirnsic_calibration`

***Step 2:***

The program will show every image with found chessboard corners

Press any key to continue:

<img src="./doc/corners.png" width = 50% height=50% align=left>

The console will print the distribution suggestion of **X, Y, Size, Skew** of chessboard poses in images:

- X: the horizontal distribution of chessboard poses in images
- Y: the vertical distribution of chessboard poses in images
- Size: the square of corners / image square 
- Skew: different angles

The bigger and greener the number, the better distribution of poses.

<img src="./doc/mono_calib.gif" width = 70% height=70% align=left>

***Step 3:***

Just wait when the terminal print: "**calibrating ...**"

***Step 4:***

If set `show_undistort` to "1", undistortion images are shown after calibraiton, press any key to continue:

<img src="./doc/undistort.png" width = 70% height=70% align=left>

**When use ocam model, there are more undistortion options:**

- If set `use_specified_focal_length` to "1",  `specified_fx` and `specified_fy` are used (e.g. 500*500):

<img src="./doc/f500.png" width = 70% height=70% align=left>

- If set `use_FOV` to "1", `target_FOV`  will be used to automatically calculate focal length (e.g. target FOV 80):

<img src="./doc/fov80.png" width = 70% height=70% align=left>

By default, none of the above 2 options are selected, and the focal length will be used to automatically calculate by `undistort_type`

- If `undistort_type` : **ratio** , keep aspect ratio of raw image without leaving any black borders:

<img src="./doc/ratio.png" width = 70% height=70% align=left>

- If `undistort_type` : **auto** , just leave no black boarders:

<img src="./doc/auto.png" width = 70% height=70% align=left>

***Step 5:***

The program will show the distribution plot of reprojection error of  all corners.

The denser and rounder the points distribution, the better the calibration.

For example, use pinhole (left) and fisheye(right) to calibrate the camera with FOV 100, the fisheye model provide better result:

<img src="./doc/pinhole.bmp" width = 30% height=30% align=left><img src="./doc/fisheye.bmp" width = 30% height=30% align=left>





















***Step 6:***

The program will automatically save all results

#### Result

All calibration results are saved in `./result` :

- Intrinsics file: `pinhole_intrinsics.json`  or `fisheye_intrinsics.json` or `ocam_intrinsics.txt`

- Reprojection errors file of all corners: `pixel_error.txt`

  For convenience, the script file `draw_pixel_error.py` can also visualize the error plot using matplotlib

  ​	<img style="width:350px;height:350px" src="./doc/script_visualization.png" align=left /><img style="width:350px;height:350px" src="./doc/own_visualization.bmp" align=left />

  

  

  

  

  

  

  - 

- 

- 

- 

- Plot of `pixel_error.txt` : `pixel_repro_error.bmp`

- Undistortion remap file : `undistort_remap.bin`

### View Undistortion Tool

`cd ./bin`

`./view_undistorted IMAGE_FILE INTRINSIC_FILE`

`IMAGE_FILE` : including the whole path and name of image

`INTRINSIC_FILE` : remap.bin or intrinsics file (.txt or .json)

## TODO List

- [x] Support camera model: pinhole, fisheye, ocam
- [x] Show undistorted image tool (support intrinsics files or remap.bin)
- [x] Show reprojection errors plot of pixels of each image
- [x] Output existing intrinsics file format
- [x] If ocam, support automatic undistort focal length (auto, ratio)
- [x] If ocam, support input specified undistort focal length or FOV
- [x] Speed up finding chessboard corners
- [x] Identify suitable calibration pose including X, Y, size, skew
- [x] Reimplement ocam model algrithm which can estimate c, d, e
- [ ] GUI such as QT

## Reference

ROS camera calibration: http://github.com/ros-perception/image_pipeline

OpenCV Fisheye camera model: http://docs.opencv.org

Ocam model C++ implement: http://github.com/siposcsaba89/ocam-calib-cpp

## Manage

"caiyongkai@nullmax.ai" manage this repo.

