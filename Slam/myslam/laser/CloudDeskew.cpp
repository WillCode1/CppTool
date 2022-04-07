#include "utility.h"
#include "EigenRotation.h"

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)
    (float, intensity, intensity)(uint16_t, ring, ring)(float, time, time))

struct OusterPointXYZIRT
{
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
    (uint32_t, t, t)(uint16_t, reflectivity, reflectivity)
    (uint8_t, ring, ring)(uint16_t, noise, noise)(uint32_t, range, range))

// Use the Velodyne point format as a common representation
using PointXYZIRT = VelodynePointXYZIRT;

const int queueLength = 2000;

class CloudDeskewByImuPreintegration
{
private:
    int N_SCAN;
    int Horizon_SCAN;
    int downsampleRate;
    float lidarMinRange;
    float lidarMaxRange;

    std::mutex imuLock;
    std::mutex odoLock;

    std::deque<ImuData> imuQueue;
    std::deque<OdometryData> odomQueue;
    std::deque<PointCloudData<PointXYZIRT>> cloudQueue;

    double *imuTime = new double[queueLength];
    double *imuRotX = new double[queueLength];
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];

    int imuPointerCur;
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<PointType>::Ptr fullCloud;
    pcl::PointCloud<PointType>::Ptr extractedCloud;

    cv::Mat rangeMat;

    bool odomDeskewFlag;
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;

    double timeScanCur;
    double timeScanEnd;

    bool imuAvailable;
    bool odomAvailable;

    /*
        // 需要修正外参，把imu数据转换到lidar坐标系
        void imuHandler(const ImuData &imuMsg);

        // 订阅imu里程计，由imuPreintegration积分计算得到的每时刻imu位姿.
        void odometryHandler(const OdometryData &odometryMsg);

        // 检查是否存在time通道!!
        void cloudHandler(const PointCloudData<PointXYZIRT> &laserCloudMsg);
    */

public:
    CloudDeskewByImuPreintegration()
    {
        N_SCAN = 16;
        Horizon_SCAN = 1800;
        downsampleRate = 1;
        lidarMinRange = 1.0;
        lidarMaxRange = 1000.0;

        //分配内存
        allocateMemory();
        //重置部分参数
        resetParameters();
        // setVerbosityLevel用于设置控制台输出的信息。(pcl::console::L_ALWAYS)不会输出任何信息;L_DEBUG会输出DEBUG信息;
        // L_ERROR会输出ERROR信息
        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }

    void allocateMemory()
    {
        //根据params.yaml中给出的N_SCAN Horizon_SCAN参数值分配内存
        //用智能指针的reset方法在构造函数里面进行初始化
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        fullCloud.reset(new pcl::PointCloud<PointType>());
        extractedCloud.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(N_SCAN * Horizon_SCAN);

        resetParameters();
    }

    void resetParameters()
    {
        //清零操作
        laserCloudIn->clear();
        extractedCloud->clear();
        // reset range matrix for range image projection,
        //初始全部用FLT_MAX 填充，
        //因此后文函数projectPointCloud中有一句if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX) continue;
        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

        imuPointerCur = 0;
        firstPointFlag = true;
        odomDeskewFlag = false;

        for (int i = 0; i < queueLength; ++i)
        {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
        }
    }

    ~CloudDeskewByImuPreintegration() {}

    /**
     * 订阅原始imu数据
     * 1、imu原始测量数据转换到lidar系，加速度、角速度、RPY
     */
    void imuHandler(const ImuData &imuMsg)
    {
        ImuData thisImu;
        // imuConverter在头文件utility.h中，作用是把imu数据转换到lidar坐标系
        // thisImu = Tool::imuConverter(imuMsg);
        std::lock_guard<std::mutex> lock1(imuLock);
        imuQueue.push_back(thisImu);
    }

    /**
     * 订阅imu里程计，由imuPreintegration积分计算得到的每时刻imu位姿.(地图优化程序中发布的)
     */
    void odometryHandler(const OdometryData &odometryMsg)
    {
        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(odometryMsg);
    }

    // 检查是否存在time通道!!
    void cloudHandler(const PointCloudData<PointXYZIRT> &laserCloudMsg)
    {
        //添加一帧激光点云到队列，取出最早一帧作为当前帧，计算起止时间戳，检查数据有效性
        if (!cachePointCloud(laserCloudMsg))
            return;
        // 当前帧起止时刻对应的imu数据、imu里程计数据处理
        if (!deskewInfo())
            return;

        // 当前帧激光点云运动畸变校正
        // 1、检查激光点距离、扫描线是否合规
        // 2、激光运动畸变校正，保存激光点
        projectPointCloud();

        // 提取有效激光点，存extractedCloud
        cloudExtraction();

        // 发布当前帧校正后点云，有效点
        // publishClouds();

        // 重置参数，接收每帧lidar数据都要重置这些参数
        resetParameters();
    }

    bool cachePointCloud(const PointCloudData<PointXYZIRT> &laserCloudMsg)
    {
        // get timestamp
        //这一点的时间被记录下来，存入timeScanCur中，函数deskewPoint中会被加上laserCloudIn->points[i].time
        timeScanCur = laserCloudMsg.stamp;
        //可以看出lasercloudin中存储的time是一帧中距离起始点的相对时间,timeScanEnd是该帧点云的结尾时间
        timeScanEnd = timeScanCur + laserCloudMsg.sweepCloudPtr->back().time;

        cloudQueue.push_back(laserCloudMsg);
        if (cloudQueue.size() <= 2)
            return false;

        laserCloudIn = cloudQueue.front().sweepCloudPtr;
        cloudQueue.pop_front();

        return true;
    }

    bool deskewInfo()
    {
        std::lock_guard<std::mutex> lock1(imuLock);
        std::lock_guard<std::mutex> lock2(odoLock);

        // make sure IMU data available for the scan
        // 要求imu数据时间上包含激光数据，否则不往下处理了
        if (imuQueue.empty() || imuQueue.front().stamp > timeScanCur || imuQueue.back().stamp < timeScanEnd)
        {
            std::cout << "Waiting for IMU data ..." << std::endl;
            return false;
        }

        // 当前帧对应imu数据处理
        // 1、遍历当前激光帧起止时刻之间的imu数据，初始时刻对应imu的姿态角RPY设为当前帧的初始姿态角
        // 2、用角速度、时间积分，计算每一时刻相对于初始时刻的旋转量，初始时刻旋转设为0
        // 注：imu数据都已经转换到lidar系下了
        // imu去畸变参数计算
        imuDeskewInfo();

        // 当前帧对应imu里程计处理
        // 1、遍历当前激光帧起止时刻之间的imu里程计数据，初始时刻对应imu里程计设为当前帧的初始位姿
        // 2、用起始、终止时刻对应imu里程计，计算相对位姿变换，保存平移增量
        // 注：imu数据都已经转换到lidar系下了
        //里程计去畸变参数计算
        odomDeskewInfo();

        return true;
    }

    void imuDeskewInfo()
    {
        imuAvailable = false;

        // 从imu队列中删除当前激光帧0.01s前面时刻的imu数据
        while (!imuQueue.empty())
        {
            if (imuQueue.front().stamp < timeScanCur - 0.01)
                imuQueue.pop_front();
            else
                break;
        }

        if (imuQueue.empty())
            return;

        imuPointerCur = 0;
        // 遍历当前激光帧起止时刻（前后扩展0.01s）之间的imu数据
        for (int i = 0; i < (int)imuQueue.size(); ++i)
        {
            const ImuData& thisImuMsg = imuQueue[i];
            double currentImuTime = thisImuMsg.stamp;

            // 超过当前激光帧结束时刻0.01s，结束
            if (currentImuTime > timeScanEnd + 0.01)
                break;

            // 第一帧imu旋转角初始化
            if (imuPointerCur == 0)
            {
                imuRotX[0] = 0;
                imuRotY[0] = 0;
                imuRotZ[0] = 0;
                imuTime[0] = currentImuTime;
                ++imuPointerCur;
                continue;
            }

            // get angular velocity
            // 提取imu角速度
            double angular_x = thisImuMsg.angular_velocity.x();
            double angular_y = thisImuMsg.angular_velocity.y();
            double angular_z = thisImuMsg.angular_velocity.z();

            // integrate rotation
            double timeDiff = currentImuTime - imuTime[imuPointerCur - 1];
            imuRotX[imuPointerCur] = imuRotX[imuPointerCur - 1] + angular_x * timeDiff;
            imuRotY[imuPointerCur] = imuRotY[imuPointerCur - 1] + angular_y * timeDiff;
            imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur - 1] + angular_z * timeDiff;
            imuTime[imuPointerCur] = currentImuTime;
            ++imuPointerCur;
        }

        --imuPointerCur;
        // 没有合规的imu数据
        if (imuPointerCur <= 0)
            return;

        imuAvailable = true;
    }

    //初始pose信息保存在cloudInfo里
    void odomDeskewInfo()
    {
        odomAvailable = false;
        // 从imu里程计队列中删除当前激光帧0.01s前面时刻的imu数据
        while (!odomQueue.empty())
        {
            if (odomQueue.front().stamp < timeScanCur - 0.01)
                odomQueue.pop_front();
            else
                break;
        }

        if (odomQueue.empty())
            return;

        // 要求必须有当前激光帧时刻之前的里程计数据
        if (odomQueue.front().stamp > timeScanCur)
            return;

        // get start odometry at the beinning of the scan(地图优化程序中发布的)
        OdometryData startOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            startOdomMsg = odomQueue[i];
            // 在cloudHandler的cachePointCloud函数中，timeScanCur = cloudHeader.stamp.toSec();，即当前帧点云的初始时刻
            //找到第一个大于初始时刻的odom
            if (startOdomMsg.stamp < timeScanCur)
                continue;
            else
                break;
        }

        // 提取imu里程计姿态角
        Eigen::Vector3d rpy = Eigen_Tool::EigenRotation::Quaternion2RPY(startOdomMsg.pose.orientation);

        odomAvailable = true;

        // get end odometry at the end of the scan
        odomDeskewFlag = false;
        // 如果当前激光帧结束时刻之后没有imu里程计数据，返回
        if (odomQueue.back().stamp < timeScanEnd)
            return;

        OdometryData endOdomMsg;
        // 提取当前激光帧结束时刻的imu里程计
        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            endOdomMsg = odomQueue[i];
            // 在cloudHandler的cachePointCloud函数中，       timeScanEnd = timeScanCur + laserCloudIn->points.back().time;
            // 找到第一个大于一帧激光结束时刻的odom
            if (endOdomMsg.stamp < timeScanEnd)
                continue;
            else
                break;
        }
#if 1
        //感觉之后计算的信息并没有被用到
        Eigen::Affine3f transBegin =
            pcl::getTransformation(startOdomMsg.pose.position.x(), startOdomMsg.pose.position.y(), startOdomMsg.pose.position.z(), rpy(0), rpy(1), rpy(2));

        rpy = Eigen_Tool::EigenRotation::Quaternion2RPY(endOdomMsg.pose.orientation);
        Eigen::Affine3f transEnd =
            pcl::getTransformation(endOdomMsg.pose.position.x(), endOdomMsg.pose.position.y(), endOdomMsg.pose.position.z(), rpy(0), rpy(1), rpy(2));

        // 起止时刻imu里程计的相对变换
        Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

        // 相对变换，提取增量平移、旋转（欧拉角）
        float rollIncre, pitchIncre, yawIncre;

        // 给定的转换中，提取XYZ以及欧拉角,通过tranBt 获得增量值  后续去畸变用到
        pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);
#endif
        odomDeskewFlag = true;
    }

    /**
     * 在当前激光帧起止时间范围内，计算某一时刻的旋转（相对于起始时刻的旋转增量）
     */
    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
    {
        *rotXCur = 0;
        *rotYCur = 0;
        *rotZCur = 0;
        // 查找当前时刻在imuTime下的索引
        int imuPointerFront = 0;
        // imuDeskewInfo中，对imuPointerCur进行计数(计数到超过当前激光帧结束时刻0.01s)
        while (imuPointerFront < imuPointerCur)
        {
            // imuTime在imuDeskewInfo（deskewInfo中调用，deskewInfo在cloudHandler中调用）被赋值，从imuQueue中取值
            // pointTime为当前时刻，由此函数的函数形参传入,要找到imu积分列表里第一个大于当前时间的索引
            if (pointTime < imuTime[imuPointerFront])
                break;
            ++imuPointerFront;
        }
        // 设为离当前时刻最近的旋转增量
        //如果计数为0或该次imu时间戳小于了当前时间戳(异常退出)
        if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
        {
            //未找到大于当前时刻的imu积分索引
            // imuRotX等为之前积分出的内容.(imuDeskewInfo中)
            *rotXCur = imuRotX[imuPointerFront];
            *rotYCur = imuRotY[imuPointerFront];
            *rotZCur = imuRotZ[imuPointerFront];
        }
        else
        {
            // 前后时刻插值计算当前时刻的旋转增量
            //此时front的时间是大于当前pointTime时间，back=front-1刚好小于当前pointTime时间，前后时刻插值计算
            int imuPointerBack = imuPointerFront - 1;
            //算一下该点时间戳在本组imu中的位置
            double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            //这三项作为函数返回值，以形参指针的方式返回
            //按前后百分比赋予旋转量
            *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
            *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
            *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
        }
    }

    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur)
    {
        // 如果传感器移动速度较慢，例如人行走的速度，那么可以认为激光在一帧时间范围内，平移量小到可以忽略不计
        *posXCur = 0;
        *posYCur = 0;
        *posZCur = 0;

        // If the sensor moves relatively slow, like walking speed, positional deskew seems to have little benefits. Thus code below is commented.

        if (odomAvailable == false || odomDeskewFlag == false)
            return;

        float ratio = relTime / (timeScanEnd - timeScanCur);

        *posXCur = ratio * odomIncreX;
        *posYCur = ratio * odomIncreY;
        *posZCur = ratio * odomIncreZ;
    }

    /**
     * 激光运动畸变校正
     * 利用当前帧起止时刻之间的imu数据计算旋转增量，imu里程计数据计算平移增量，进而将每一时刻激光点位置变换到第一个激光点坐标系下，进行运动补偿
     */
    // relTime:laserCloudIn->points[i].time
    PointType deskewPoint(PointType *point, double relTime)
    {
        //这个来源于上文的时间戳通道和imu可用判断，没有或是不可用则返回点
        if (!imuAvailable)
            return *point;

        //点的时间等于scan时间加relTime（后文的laserCloudIn->points[i].time）
        // lasercloudin中存储的time是一帧中距离起始点的相对时间
        // 在cloudHandler的cachePointCloud函数中，timeScanCur = cloudHeader.stamp.toSec();，即当前帧点云的初始时刻
        //二者相加即可得到当前点的准确时刻
        double pointTime = timeScanCur + relTime;

        // 求解关于旋转和平移的运动畸变
        //根据时间戳插值获取imu计算的旋转量与位置量（注意imu计算的相对于起始时刻的旋转增量）
        float rotXCur, rotYCur, rotZCur;
        findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

        float posXCur, posYCur, posZCur;
        findPosition(relTime, &posXCur, &posYCur, &posZCur);

        //这里的firstPointFlag来源于resetParameters函数，而resetParameters函数每次ros调用cloudHandler都会启动
        // 第一个点的位姿增量（0），求逆
        if (firstPointFlag)
        {
            transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
            firstPointFlag = false; //改成false以后，同一帧激光数据的下一次就不执行了
        }

        // transform points to start
        //扫描当前点时lidar的世界坐标系下变换矩阵
        Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
        //扫描该点相对扫描本次scan第一个点的lidar变换矩阵=
        //第一个点时lidar世界坐标系下变换矩阵的逆×当前点时lidar世界坐标系下变换矩阵
        // Tij=Twi^-1 * Twj
        //注：这里准确的来说，不是世界坐标系,
        //根据代码来看，是把imu积分:
        //从imuDeskewInfo函数中，在当前激光帧开始的前0.01秒的imu数据开始积分，
        //把它作为原点，然后获取当前激光帧第一个点时刻的位姿增量transStartInverse，
        //和当前点时刻的位姿增量transFinal，根据逆矩阵计算二者变换transBt。
        //因此相对的不是“世界坐标系”，
        //而是“当前激光帧开始前的0.01秒的雷达坐标系（在imucallback函数中已经把imu转换到了雷达坐标系了）
        Eigen::Affine3f transBt = transStartInverse * transFinal;

        PointType newPoint;

        //根据lidar位姿变换 Tij，修正点云位置: Tij * Pj
        newPoint.x = transBt(0, 0) * point->x + transBt(0, 1) * point->y + transBt(0, 2) * point->z + transBt(0, 3);
        newPoint.y = transBt(1, 0) * point->x + transBt(1, 1) * point->y + transBt(1, 2) * point->z + transBt(1, 3);
        newPoint.z = transBt(2, 0) * point->x + transBt(2, 1) * point->y + transBt(2, 2) * point->z + transBt(2, 3);
        newPoint.intensity = point->intensity;

        return newPoint;
    }

    void projectPointCloud()
    {
        int cloudSize = laserCloudIn->points.size(); //点云数据量 用于下面一个个点投影
        // range image projection
        for (int i = 0; i < cloudSize; ++i)
        {
            PointType thisPoint;
            // laserCloudIn就是原始的点云话题中的数据
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            thisPoint.intensity = laserCloudIn->points[i].intensity;

            float range = Tool::pointDistance(thisPoint);
            if (range < lidarMinRange || range > lidarMaxRange)
                continue;
            //距离图像的行  与点云中ring对应,
            // rowIdn计算出该点激光雷达是水平方向上第几线的。从下往上计数，-15度记为初始线，第0线，一共16线(N_SCAN=16
            int rowIdn = laserCloudIn->points[i].ring;
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            if (rowIdn % downsampleRate != 0)
                continue;

            //水平角分辨率
            float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
            // Horizon_SCAN=1800,每格0.2度
            static float ang_res_x = 360.0 / float(Horizon_SCAN);

            // horizonAngle 为[-180,180],horizonAngle -90 为[-270,90],-round 为[-90,270], /ang_res_x 为[-450,1350]
            //+Horizon_SCAN/2为[450,2250]
            //  即把horizonAngle从[-180,180]映射到[450,2250]
            int columnIdn = -round((horizonAngle - 90.0) / ang_res_x) + Horizon_SCAN / 2;
            //大于1800，则减去1800，相当于把1801～2250映射到1～450
            //先把columnIdn从horizonAngle:(-PI,PI]转换到columnIdn:[H/4,5H/4],
            //然后判断columnIdn大小，把H到5H/4的部分切下来，补到0～H/4的部分。
            //将它的范围转换到了[0,H] (H:Horizon_SCAN)。
            //这样就把扫描开始的地方角度为0与角度为360的连在了一起，非常巧妙。
            //如果前方是x，左侧是y，那么正后方左边是180，右边是-180。这里的操作就是，把它展开成一幅图:
            //                   0
            //   90                        -90
            //          180 || (-180)
            //  (-180)   -----   (-90)  ------  0  ------ 90 -------180
            //变为:  90 ----180(-180) ---- (-90)  ----- (0)    ----- 90
            if (columnIdn >= Horizon_SCAN)
                columnIdn -= Horizon_SCAN;

            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
                continue;
            //去畸变  运动补偿 这里需要用到雷达信息中的time 这个field
            thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);
            //图像中填入欧几里得深度
            rangeMat.at<float>(rowIdn, columnIdn) = range;

            // 转换成一维索引，存校正之后的激光点
            int index = columnIdn + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = thisPoint;
        }
    }

    void cloudExtraction()
    {
        // 有效激光点数量
        int count = 0;
        // extract segmented cloud for lidar odometry
        for (int i = 0; i < N_SCAN; ++i)
        {
            /// Horizon_SCAN=1800
            for (int j = 0; j < Horizon_SCAN; ++j)
            {
                if (rangeMat.at<float>(i, j) != FLT_MAX)
                {
                    // save extracted cloud
                    // 加入有效激光点
                    extractedCloud->push_back(fullCloud->points[j + i * Horizon_SCAN]);
                    // size of extracted cloud
                    ++count;
                }
            }
        }
    }
};
