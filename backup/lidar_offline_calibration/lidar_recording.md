
# 激光與相機數據採集
本文數據採集在蒙4以及RoboSense M1激光雷達上進行

## 前言
由於RoboSense激光驅動無法在PX2環境編譯, 你必須準備一台筆記本採集激光  
並在筆記本安裝RoboSense安裝激光驅動(https://github.com/RoboSense-LiDAR/ros_rslidar)  
激光數據遊筆記本採集, 圖像數據由車上Px2採集, 並利用PTP協定同步兩台機器的時間


## lidar数据采集

### px2：
1.系统设置里设置时间为从网络获取
2.啟動PTP

        sudo ptpd -gi eth0

3.查看ptpd是否开启

        ps -aux | grep ptpd 

4.檢查時間是否同步

        timedatectl set-ntp false


### 笔记本：
1. 啟動roscore

        roscore

2. 时间同步
   插好网线，网卡时间同步： 

        timedatectl set-ntp false   
        sudo ptpd -mi enp3s0

3. 查看ptpd是否开启

        ps -aux | grep ptpd 

4. 查看时间差，等待delta到0ms（大约十分钟）

        #Px2 IP有可能改動
        clockdiff 192.160.0.123 
   
5. 进入rslidar_sdk/build目录  
   設置config/config.yaml  
   將msg_source 設置為2( packet message comes from ROS or ROS2)  
   lidar_type 設置為 RSM1  
   运行程序： 
   
        ./rslidar_sdk_node

6. 显示点云
   rviz进入rviz界面，添加点云topic：add->pointcloud，设置坐标系：Frame->rslidar

7. 记录点云ros包，最后一个参数是保存的路径

        rosbag record -a -x "/rslidar_points" -o /home/lijiayun/lidar/  



## camera数据采集

1. 登录px2  
   ssh ubuntu@192.168.0.123   
   密码：ubuntu   

2. 修改配置文件保存路径

        vim auto-driving/conf/camera_config/nm_autonomous_driving.json  

    將/storage_path改為你想儲存的位置

3. 运行采集程序

        cd auto-driving/bin
        ./launch.sh start

4. 关闭时需要新开一个终端，并执行以上操作到bin目录，然后

        ./launch.sh stop
