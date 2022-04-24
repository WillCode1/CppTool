# AVP MAPPING 

----

## 1. Semantic Mapping 

There are two types of maps in HPA: 

* keyframe map ( for localization)
* quadtree occupancy map (for visualization)

### 3.1 IPM Generation 



### 3.2 Pose Estimation 

Online Pose estimation is crucial for mapping. In <sub>[2] [3] [4] </sub> online pose is generated from odometry module, which  consists of IMU (Inertial Measurement Unit) and wheel encoder (without vision). Such odometry modules can provide accurate local localization which is enough for mapping. While we do not have wheel encoder on our vehicles, Since wheel encoders is not suitable for production. The accuracy of our wheel odometry is not accurate enough, so we should use visual information when estimate vehicle odometry. 

Same as in AVP, we apply `distance transform` on the segmentation to target image probability map(distance transform). 

![](../images/prob.png)

*fig `distance transform` operation* 

Set $I_k$ is current reference keyframe and $I_i$ is current frame .

Set last frame pose is $T_{wb_{i-1}}$ . So current predicted pose is : 

$$ \overline{T_{wb_i}} =T_{wb_{i-1}}  T_{ob_{i-1}} ^{-1} T_{ob_i} $$ 

Set $p_k$ is semantic point from reference keyframe, we assume that the semantic point keeps the same gray value in different frames. 

![image-20200825163427341](../images/pose_estimation_assumption.png)

so we can use non-linear optimization method to estimate current frame pose with pose prior: 

$$T_{wb_i} = argmin(\sum_{j=0} ^{j=n}|| I_i (  \pi (  T_{wb_i} T_{wb_k} \pi^{-1}(p_{k_j})) ) - I_k(p_{k_j})||^2 + E_{prior}$$

where $\pi$ is the projection function which project point in baselink coordinate to image coordinate.  

$$E_{prior} = ||(T_{wb_i})_{xy\theta} - ( \overline{T_{wb_i}})_{xy\theta} ||_w^{2} $$

where $(\cdot)_{xy\theta}$ means the $x$,$y$,$\theta$ of the  SE2 transformation. 

### 3.3 Keyframe Connection and Generation 

The keyframe map consists of several keyframes.  The criteria for keyframe generation are very important, which may directly affect the localization result. The criteria of keyframe generation are shown blew:

1. if there is no reference keyframe (no keyframe in the map), generate a keyframe;
2. the the distance between closest keyframe and current frame is smaller than a threshold, do not generate a keyframe;
3. if the distance between reference keyframe and current frame is larger than a threshold, generate a keyframe;

In addition, keyframes retain the connection relationship when they are generated. The keyframe connect can somehow solve the ghosting problem in mapping.

![image-20200825170417351](../images/keyframe_connection.png)

*fig keyframe connection (red dot: keyframes; green line: connections)*

![image-20200825170606560](../images/keyframe_quadtree_map.png)

*fig generated quadtree occupancy map*

### 3.5 Map Generation 

Note that, we will build two type of map (keyframe map, quadtree occupancy map). It is obviously that the keyframe-map consists of keyframes, so we just need to save the keyframe images and its pose.

![](../images/keyframe_map.png) 

Once there is one keyframe generated, it is send to the mapping pipeline. We only use pixel with large value(which means those pixels close to the center of segmentation region).

Set $p_k$ is the point in keyframes, transform it to map coordinate. 

$P_m = T_{wb} \pi^{-1}(p_k) $ where $I_k(p_k) >\delta$

where $\pi$ is the projection function which project point in baselink coordinate  to image.

$\pi(P_b) = \left[ \begin{matrix}  -P_{b_y} /s + u_0 \\ -(P_{b_x} -m )/s + v_0  \end{matrix} \right]$

where $s = 15.0 (m) / 416$ (pixels)  is image scale coeffient and $m$ is translation between vehicle center and baselink.

### 3.6 Map Size 

Map length (200m), we have tested 4 routes, the map memory cost is shown below. 

| Map  Name | bin size(Mb) | vis size(Mb) |
| :-------- | :----------: | :----------: |
| hpa map 1 |     28.6     |     15.2     |
| hpa map 2 |     24.8     |     13.4     |
| hpa map 3 |     25.1     |     11.8     |
| hpa map 4 |     26.3     |     14.1     |

