张正友标定法与两种畸变校正模型示例代码 OpenCV3.4.0 + VS2015

代码主要有两部分组成，1.张正友摄像机标定 2.畸变校正。分别包含普通摄像机模型（CV）和鱼眼摄像机模型(fisheye)下两种标定和校正方法。

各个文件简要说明
*main.cpp：主要为接口函数，以及一些接口参数的设置。
包括：patternImgPath----标定板图像存放文件夹路径（标定板图像文件名按照0.jpg, 1.jpg...如此命名）；
calibResultPath----摄像机标定内部参数文件保存路径，fx, fy, cx,cy, 以及畸变系数(k1,k2,p1,p2,k3 或者 k1,k2,k3,k4);
srcImgPath----相机拍摄测试图片保存路径，用来测试标定和畸变校正结果；
boardSize----标定板内角点行和列个数；
CCalibration类实现摄像机标定功能，calibration.run()执行读取标定板图片、角点检测、亚像素精确化、摄像机标定、计算重投影误差、保存标定参数功能；
CUndistort类实现畸变校正功能，undistort.run()执行读取内部参数、读取畸变图像、畸变校正、显示校正结果功能。

*calibration.h, calibration.cpp：实现摄像机标定，包含CV模型和Fisheye模型。默认为CV模型，如需更换为Fisheye模型，
请将calibration.h文件中的( #define CV )替换为（ #define FISHEYE ）即可，内部代码会根据宏定义来执行对应的标定和畸变校正模型代码。

*undistort.h, undistort.cpp：实现畸变校正，包含CV模型和Fisheye模型。

https://gitee.com/dingxinning/camera-calibration
https://blog.csdn.net/yuegooxi/article/details/122015346
