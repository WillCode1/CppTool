# Stereo Calibration

## 使用方法

1. 在*scripts*目录中，运行
    ./stereo_calib_build.sh

&#8195;会在 **\${PERCEPTION_ROOT}/lib/x86_64-linux** 目录下生成可执行程序 *stereo_calibration_demo*，同时在同目录下将示例图片和示例配置参数拷贝过来，示例图片放在 **data** 目录下，示例配置参数为*stereo_calib.json*。

2. 配置参数中，*image_lists* 为双目采集图片的文件名序列，按实际采集的图片文件名保存。文件名没有特殊要求，但文件序列的排列顺序必须为：
   左1.jpg
   右1.jpg
   左2.jpg
   右2.jpg
   ...

&#8195;*chessboard*为标定棋盘格的信息，其中*height*和*width*分别表示棋盘格内角点高度方向和宽度方向的个数，*square_size*为每个小方格实际物理尺寸，以米为单位。

3. 运行 *stereo_calibration_demo* 则会读取同目录下配置*stereo_calib.json*以及*data*下的双目标定图片。

&#8195;运行成功后，会提示匹配成功的双目图像个数以及标定精度打印到屏幕上，如下图所示：
<div align="center">
  <img src="images/1.png">
</div>
<div align="center">
  图1. 双目标定运行成功及精度提示
</div>

4. 基于解算好的标定参数，程序会将所有双目图片进行去畸变，并进行核线对齐，示例如下图所示：
<div align="center">
  <img src="images/2.png">
</div>
<div align="center">
  图2. 双目核线对齐
</div>
&#8195;标定参数正常的情况下，两个相机图像上的同一个目标应落在同一条绿线上，用以检验双目标定精度。按空格跳下一对双目图片进行查看，查看完毕后程序自动关闭。

5. 标定结果会保存到同目录的*extrinsics.yml*和*intrinsics.yml*中，分别保存双目相机的 *外参* (以左相机为基准，右相机相对于左相机的旋转平移，以及两个相机相对于平行基准轴的旋转阵)和 *内参* (两个相机各自的焦距，主点位置和畸变参数)
