# Front camera calibration using lane marking

## 1.How to use these code
  Environment:  
  [opencv](https://docs.opencv.org/3.1.0/d7/d9f/tutorial_linux_install.html)(3.2.0 or later) and 
  [ceres](http://ceres-solver.org/installation.html)  


	step1: modify picture Data_Path ./config/config.yml at line 77
           eg:Data_Path: /home/user/Nullmaxproject/good/frame_vc1_*.bmp,modify this to your own data path. The "*" symbols represents the holder of image ID            

    step2: modify the Image_Sequences field at line 81. It represents the starting and ending id of each input image sequence.

	step3: enter this folder,create compile dir,
           eg: cd front-camera-calibration-master;mkdir build;cd build
	step4: compile,
           eg: cmake ..;make
	step5: run below code in terminal,
           eg: front-camera-calibration ../config
	then,you will find result pitch and yaw value in terminal.

	
  Description:

	note1:config dir contain two configuration files,mkz.yml and Roi_para.yaml.  
	note2:mkz.yml contain camera inner-parameter fx,fy,cx,cy,heigth,you can modify these value depend yourself camera.  
		  mkz.yml contain Data_Path: /home/user/Nullmaxproject/good/*.bmp.You need to modify in yourself img dir.  
	note3:Roi_para.yaml contain Roi1 and Roi2 parameters,you can modify them according to description in the file.
	note4: you can modify k range to select specific slope line from the detected lines.

  1.行驶及道路要求要求:
  ①道路尽量为直道,无弯道
  ②车道线清晰,可视范围内至少3根车道线
  ③车辆行驶速度40km/h以上,尽量不换道
  2.图像要求
  ①图像保存原图
  ②相机的内参信息
  ③相机安装高度(相对地面)
  ④采集图片300以上

  0.针孔和鱼眼、ocam相机都有，一般只用针孔
  1.获取图片，选取非弯道图片
  2.配置文件里填入相机内参,相机高度
  3.calibration_result.yaml文件输出rpy

## 2.Abstract of algorithm
The figure below describe the algorithm flow.   
![](doc/images/alg_0.png)   

The core function that implement in the code list below:    
- **Detect left/right lane**    
```
// front-camera-calibration/include/calculate_k_b.h

void calculate_k_b(const cv::Mat img_my, const Roi_data &roi_data,
                   float *lane_kb, const int &canny_threadhold) {
  ...
}
```
- **Calc vanishing point**
- **Calc raw pitch**
```
// front-camera-calibration/src/utils.cpp

cv::Point2f CalculateRawPitch(const Para_data &data_set, Lane_Data &lane_data) {
  ...
}

```
- **Optimize pitch & yaw**
```
// front-camera-calibration/src/utils.cpp

void OptimizePitch_Yaw(const std::vector<Lane_Data> &lane_data,
                       const Para_data &data_set, float &pitch_optimize,
                       float &yaw_optimize) {
  ...  
}
```
