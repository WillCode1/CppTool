# Front Camera Extrinsic Adjustment

## 一、项目简介
```
  手動調整相機外參的角度, 搭配Calibration模塊使用(主要調整Roll角)
```
## 二、运行环境
```
  操作系统: Ubuntu16.04, Ubuntu 18.04 x86_64
 
  编程语言：C++
 
  依赖库： libopencv-dev
```
## 三、准备Config
```
Adjustment模塊與Calibration一樣依賴於config.yaml, 你可以參考data/config.yaml的範例, 修改Adjustment_Image_path欄位即可  
合適的圖片應該在車輛前方能看見一道水平線作為參照物(你可以在下面找到範例圖像)
如果你已經有先驗的Row, Yaw, Pitch, 也可以在 Angle_pitch, Angle_yaw, Angle_roll三個欄位填寫  

如果搭配Calibration模塊使用, 那麼可以利用其輸出的calibration_result.yaml 來取得Yaw,Pitch,  你可以在下面運行範例中找到相關的操作方法
```

## 四、如何运行

### 使用config
```
$ cd build/bin
# 將config作為的目錄作為輸入
$ ./front-camera-extrinsic-adjustment-sample ../../data/
```

### 使用config並從calibration模塊的輸出讀取旋轉

```
$ cd build/bin
# 將calibration.yaml(由calibration模塊輸出)作為第二個輸入
$ ./front-camera-extrinsic-adjustment-sample ../../data/ ../../data/calibration.yaml
```

## 五, 标定流程
```
 1. 選定前方含有水平線的圖像並修改config.yaml中的Adjustment_Image_Path
 2. 執行front-camera-extrinsic-adjustment-sample(細節參考第4節(如何運行))
 3. 圖像以及對應的鳥瞰圖會彈出
 4. 利用鍵盤調整Pitch(p鍵,o鍵), Roll(r鍵,e鍵), Yaw(y鍵,t鍵), 使得鳥瞰圖中的水平線確實看起來水平(Terminal中會提示鍵盤與旋轉角的對應)
  
```
## 六, 圖像範例

### 合適的輸入圖像 - 前方含有水平線作為參照
![](../data/mkz_0231/frame_vc1_6280.jpg)

### 操作介面展示
![](./images/adjustment_interface.png)


