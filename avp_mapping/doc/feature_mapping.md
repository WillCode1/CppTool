# Feature Mapping

----

Feature Mapping is part of AVP/HPA mapping system. It runs after semantic mapping, since it requires some trajectory log file generated from semantic mapping process.

![](feature_mapping/feature_mapping.png)

## 1.  Feature Mapping API Overview 

![](feature_mapping/feature_mapping-API.png)

## 2. Feature Mapping Pseudo Code 

Require: 4 cam panoramic images, trajectory log, odometry log

output: feature point map

```
feature_mapper = CreateFeatureMapper(...)

while(剩余图片>0)
   feature_mappe->TrackMultiCam(
         img_left[i],img_front[i],img_right[i],img_back[i],odom_log[i])

end while 

feature_mapper->AlignMapToTrajectory（trajectory_log)
feature_mapper->SaveMap(...)
```

You can also  refer to the code(`nm_feature_mapping`)  in demo



## 3. Input Data 

### 3.1 Panoramic (Equirectangular) Image

Since raw fisheye image is not suitable for feature extraction and point projection, panoramic (equirectangular) images are used.  This is widely used in 360 camera slam system, e.g. openvslam <sub>[1]</sub>

You can find more equirectangular projection  information in wiki<sub>[2]</sub>

![](feature_mapping/fisheye2pano.png)

Point projection: 

$p_{uv} =\left[ \begin{matrix}  u \\ v \end{matrix} \right] = \pi(P)$, $\hat{P} = \frac{P}{|P| }$

$lat =-asin(\hat{P}_y)$ , $lon = atan(\frac{P_z}{P_x})$

$$\left[ \begin{matrix} u \\ v  \end{matrix} \right] = \left[ \begin{matrix} k *  (-lon + \pi ) \\  k *( -lat + \frac{\pi}{2}) - topcut \end{matrix} \right]$$ 

其中$k =\frac{w}{\pi} $ ,$w$为图片的宽度, topcut是图片预处理剪裁的大小，因为我们使用的鱼眼相机HFov只有~90°，因此会在图像的上下处留有大片的黑边，这些信息没有使用意义因此我们选择直接裁切丢弃.

###  3.1 Trajctory Log and Odoemtry Log

Trajectory log 是语义建图阶段产生的车辆运行的轨迹,这个trajectory log的作用主要有两个:

1. 提供给规划控制，作为车辆学习的路径
2. 作为feature mapping的一个参考轨迹，feature mapping的特征地图需要对齐到这一个轨迹上才能使得feature point地图和semantic 坐标系对齐

**Trajectory log** format: 

我们使用TUM的轨迹格式输出trajectory log

```
timestmap(s)  x y z  qx qy qz qw
```

同样的语义建图阶段还会产生odometry log文件，其记录了图像对应的里程计信息．

## 4. System Initialization 

系统初始化的主要目的就是计算两帧之间的相对位姿并构建一个初始化地图．同时需要确保初始化的地图的尺度是正确的．

正如我们所知，单目相机是无法计算真实尺度的，但是对于我们的系统用于多个相机，理论上系统的尺度是可以估计出来的．然而初始化的时候系统的真实尺度是比较难以计算的，错误估计的尺度还会影响后续建图的精度和稳定性，因此我们需要车辆里程计的信息来辅助进行初始化．

一般visual slam的初始化过程都是通过计算两帧之间本质矩阵(Essential Matrix)并将其分解获得相对位姿的，然后这样计算的成功率不高，主要受限于:

1. 特征点的误匹配
2. 特征点的精度
3. 分解pose结果差异化判断比较严苛

为了让系统初始化更加容易，同时充分利用车辆运动的特征点（平面运动）．我们采用里程计信息作为初始化的重要依据，视觉负责校验初始化的正确性．

设里程计信息位$T_o$ ，则$i$,$j$时刻的车辆里程计分别为$T_{o_i}$, $T_{o_j}$.需要注意的是原始的车辆里程计信息是SO2，我们在使用的过程中需要将其转化为SO3，即$T_o \in SO3$ ．则$i$,$j$时刻相机之间的相对于运动为: 

$$T_{c_{ij}} =( T_{o_i}  T_{oc}) ^{-1} ( T_{o_j}  T_{oc}) $$

然后通过一下校验判断当前的特征点和初始化pose是否满足要求: 

1. 有效点(可三角化特征点)数量
2. 视差角大小

若通过这两个条件，则认为当前计算的特征点和相对pose满足初始化条件．若不通过则使用下一帧继续进行初始化．



## 5. Track 

`Track`流程主要是计算图像的pose，通过生成关键帧用于建图．主要是内容在一下几个函数中:

1. TrackWithMotionModel
2. TrackReferenceKeyFrame
3. TrackLocalMap

这些函数跟ORB-SLAM2中定义的类似，不同在于优化的过程中是multi-cam optimzation的方式，这里就不具体展开了．

## 6. Mapping 

`Mapping`就是利用当前的keyframe和过去的部分keyframs一起三角化特征点，同时利用`local bundle adjustment`来优化局部地图．

需要注意的是在三角化(triangulation)方法中，有一种比较特殊: 

设$p_1$, $p_2$ 是两个keyframe上对应的特征点的归一化坐标($z=1$)，并且keyframe之间的相对pose为$T=T_{21}= \left[ \begin{matrix} R \space t \end{matrix} \right] $ ，求$p_1$, $p_2$的深度值$z_1$ $z_2$

$$z_2  p_2 = T_{21}  (z_1 p_1) = z_1 R p_1 + t $$

$$\left[  \begin{matrix} p_2 &&-R p_1  \end{matrix}\right] \left[  \begin{matrix} z_2 \\ z_1 \end{matrix}\right] =t $$

$$ \left[  \begin{matrix}  p_2 \\ R p_1 \end{matrix} \right] \left[  \begin{matrix} p_2 &&-R p_1  \end{matrix}\right] \left[  \begin{matrix} z_2 \\ z_1 \end{matrix}\right] =\left[  \begin{matrix}  p_2 \\ R p_1 \end{matrix} \right] t $$

$$\left[ \begin{matrix}  p_2 p_2 && -p_2 Rp_1 \\ p_2 Rp_1 && -Rp_1 Rp_1  \end{matrix}\right]\left[  \begin{matrix} z_2 \\ z_1 \end{matrix}\right] = \left[  \begin{matrix}  p_2 \\ R p_1 \end{matrix} \right] t$$

$$\left[  \begin{matrix} z_2 \\ z_1 \end{matrix}\right] =\left[ \begin{matrix}  p_2 p_2 && -p_2 Rp_1 \\ p_2 Rp_1 && -Rp_1 Rp_1  \end{matrix}\right]^{-1} \left[  \begin{matrix}  p_2 \\ R p_1 \end{matrix} \right] t $$

则三角化的点坐标为  $p_1 z_1$, $p_2 z_2$

## Reference 

[1] https://github.com/xdspacelab/openvslam

[2] https://en.wikipedia.org/wiki/Equirectangular_projection





