#include "utility.h"
#include <pcl/filters/voxel_grid.h>

using PointType = pcl::PointXYZI;

struct smoothness_t
{
    float value; // 曲率值
    size_t ind;  // 激光点一维索引
};

/**
 * 曲率比较函数，从小到大排序
 */
struct by_value
{
    bool operator()(smoothness_t const &left, smoothness_t const &right)
    {
        return left.value < right.value;
    }
};

// 用来提取角点、平面点
class FeatureExtraction
{

public:
    int N_SCAN;
    int Horizon_SCAN;
    float edgeThreshold;
    float surfThreshold;
    float odometrySurfLeafSize;

    // 当前激光帧运动畸变校正后的有效点云
    pcl::PointCloud<PointType>::Ptr extractedCloud;
    // 当前激光帧角点点云集合
    // pcl::PointCloud<PointType>::Ptr cornerCloud;
    // 当前激光帧平面点点云集合
    // pcl::PointCloud<PointType>::Ptr surfaceCloud;

    pcl::VoxelGrid<PointType> downSizeFilter;

    std::vector<smoothness_t> cloudSmoothness;
    //用来做曲率计算的中间变量
    float *cloudCurvature = nullptr;
    // 特征提取标记，1表示遮挡、平行，或者已经进行特征提取的点，0表示还未进行特征提取处理
    int *cloudNeighborPicked = nullptr;
    // 1表示角点，-1表示平面点
    int *cloudLabel = nullptr;

    FeatureExtraction(int n_scan, int horizon_scan, float edgeThreshold_, float surfThreshold_, float odometry_surfLeafSize)
        : N_SCAN(n_scan), Horizon_SCAN(horizon_scan),
          edgeThreshold(edgeThreshold_), surfThreshold(surfThreshold_),
          odometrySurfLeafSize(odometry_surfLeafSize)
    {
        initializationValue();
    }

    ~FeatureExtraction()
    {
        if (cloudCurvature)
        {
            delete[] cloudCurvature;
            cloudCurvature = nullptr;
        }
        if (cloudNeighborPicked)
        {
            delete[] cloudNeighborPicked;
            cloudNeighborPicked = nullptr;
        }
        if (cloudLabel)
        {
            delete[] cloudLabel;
            cloudLabel = nullptr;
        }
    }

    void initializationValue()
    {
        cloudSmoothness.resize(N_SCAN * Horizon_SCAN);

        downSizeFilter.setLeafSize(odometrySurfLeafSize, odometrySurfLeafSize, odometrySurfLeafSize);

        extractedCloud.reset(new pcl::PointCloud<PointType>());
        // cornerCloud.reset(new pcl::PointCloud<PointType>());
        // surfaceCloud.reset(new pcl::PointCloud<PointType>());

        cloudCurvature = new float[N_SCAN * Horizon_SCAN];
        cloudNeighborPicked = new int[N_SCAN * Horizon_SCAN];
        cloudLabel = new int[N_SCAN * Horizon_SCAN];
    }

    //接收imageProjection.cpp中发布的去畸变的点云，实时处理的回调函数
    void laserCloudHandler(pcl::PointCloud<PointType>::Ptr cloud_deskewed, const std::vector<float> &pointRange,
                           const std::vector<int> &pointColInd, const std::vector<int> &startRingIndex, const std::vector<int> &endRingIndex,
                           pcl::PointCloud<PointType>::Ptr cornerCloud, pcl::PointCloud<PointType>::Ptr surfaceCloud)
    {
        // 获取的去畸变点云信息
        extractedCloud = cloud_deskewed;

        // 计算当前激光帧点云中每个点的曲率
        calculateSmoothness(pointRange);

        // 标记属于遮挡、平行两种情况的点，不做特征提取
        markOccludedPoints(pointRange, pointColInd);

        // 点云角点、平面点特征提取
        // 1、遍历扫描线，每根扫描线扫描一周的点云划分为6段，针对每段提取20个角点、不限数量的平面点，加入角点集合、平面点集合
        // 2、认为非角点的点都是平面点，加入平面点云集合，最后降采样
        extractFeatures(startRingIndex, endRingIndex, pointColInd, cornerCloud, surfaceCloud);
    }

    void calculateSmoothness(const std::vector<float> &pointRange)
    {
        // 遍历当前激光帧运动畸变校正后的有效点云
        int cloudSize = extractedCloud->points.size();
        for (int i = 5; i < cloudSize - 5; i++)
        {
            // 用当前激光点前后5个点计算当前点的曲率
            //注意，这里把前后五个点共10个点加起来，还减去了10倍的当前点
            float diffRange = pointRange[i - 5] +
                              pointRange[i - 4] +
                              pointRange[i - 3] +
                              pointRange[i - 2] +
                              pointRange[i - 1] -
                              pointRange[i] * 10 +
                              pointRange[i + 1] +
                              pointRange[i + 2] +
                              pointRange[i + 3] +
                              pointRange[i + 4] +
                              pointRange[i + 5];
            // 距离差值平方作为曲率
            cloudCurvature[i] = diffRange * diffRange; // diffX * diffX + diffY * diffY + diffZ * diffZ;

            // 0表示还未进行特征提取处理,1表示遮挡、平行，或者已经进行特征提取的点
            cloudNeighborPicked[i] = 0;
            // 1表示角点，-1表示平面点
            cloudLabel[i] = 0;

            // 存储该点曲率值、激光点一维索引
            //之所以可以这样操作，是因为在initializationValue部分，对cloudSmoothness进行过初始化，
            //否则直接对cloudSmoothness[i]赋值，一定会报段错误
            cloudSmoothness[i].value = cloudCurvature[i];
            cloudSmoothness[i].ind = i;
        }
    }

    void markOccludedPoints(const std::vector<float> &pointRange, const std::vector<int> &pointColInd)
    {
        int cloudSize = extractedCloud->points.size();
        // mark occluded points and parallel beam points
        for (int i = 5; i < cloudSize - 6; ++i)
        {
            // 1.occluded points
            // 当前点和下一个点的range值
            float depth1 = pointRange[i];
            float depth2 = pointRange[i + 1];
            // 两个激光点之间的一维索引差值，如果在一条扫描线上，那么值为1；
            // 如果两个点之间有一些无效点被剔除了，可能会比1大，但不会特别大
            // 如果恰好前一个点在扫描一周的结束时刻，下一个点是另一条扫描线的起始时刻，那么值会很大
            int columnDiff = std::abs(int(pointColInd[i + 1] - pointColInd[i]));

            // 两个点在同一扫描线上，且距离相差大于0.3，认为存在遮挡关系
            //（也就是这两个点不在同一平面上，如果在同一平面上，距离相差不会太大）
            // 远处的点会被遮挡，标记一下该点以及相邻的5个点，后面不再进行特征提取
            if (columnDiff < 10)
            {
                // 10 pixel diff in range image
                if (depth1 - depth2 > 0.3)
                {
                    cloudNeighborPicked[i - 5] = 1;
                    cloudNeighborPicked[i - 4] = 1;
                    cloudNeighborPicked[i - 3] = 1;
                    cloudNeighborPicked[i - 2] = 1;
                    cloudNeighborPicked[i - 1] = 1;
                    cloudNeighborPicked[i] = 1;
                }
                else if (depth2 - depth1 > 0.3)
                {
                    cloudNeighborPicked[i + 1] = 1;
                    cloudNeighborPicked[i + 2] = 1;
                    cloudNeighborPicked[i + 3] = 1;
                    cloudNeighborPicked[i + 4] = 1;
                    cloudNeighborPicked[i + 5] = 1;
                    cloudNeighborPicked[i + 6] = 1;
                }
            }

            // 2.parallel beam
            // 用前后相邻点判断当前点所在平面是否与激光束方向平行
            // diff1和diff2是当前点距离前后两个点的距离
            float diff1 = std::abs(float(pointRange[i - 1] - pointRange[i]));
            float diff2 = std::abs(float(pointRange[i + 1] - pointRange[i]));
            // 如果当前点距离左右邻点都过远，则视其为瑕点，因为入射角可能太小导致误差较大
            // 选择距离变化较大的点，并将他们标记为1
            if (diff1 > 0.02 * pointRange[i] && diff2 > 0.02 * pointRange[i])
                cloudNeighborPicked[i] = 1;
        }
    }

    void extractFeatures(const std::vector<int> &startRingIndex, const std::vector<int> &endRingIndex, const std::vector<int> &pointColInd,
                         pcl::PointCloud<PointType>::Ptr cornerCloud, pcl::PointCloud<PointType>::Ptr surfaceCloud)
    {
        cornerCloud->clear();
        surfaceCloud->clear();

        pcl::PointCloud<PointType>::Ptr surfaceCloudScan(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surfaceCloudScanDS(new pcl::PointCloud<PointType>());

        for (int i = 0; i < N_SCAN; i++)
        {
            surfaceCloudScan->clear();
            // 将一条扫描线扫描一周的点云数据，划分为6段，每段分开提取有限数量的特征，保证特征均匀分布
            for (int j = 0; j < 6; j++)
            {
                // 每段点云的起始、结束索引；startRingIndex为扫描线起始第5个激光点在一维数组中的索引
                //注意：所有的点云在这里都是以"一维数组"的形式保存
                // startRingIndex和 endRingIndex 在imageProjection.cpp中的 cloudExtraction函数里被填入
                //假设 当前ring在一维数组中起始点是m，结尾点为n（不包括n），那么6段的起始点分别为：
                // m + [(n-m)/6]*j   j从0～5
                // 化简为 [（6-j)*m + nj ]/6
                // 6段的终止点分别为：
                // m + (n-m)/6 + [(n-m)/6]*j -1  j从0～5,-1是因为最后一个,减去1
                // 化简为 [（5-j)*m + (j+1)*n ]/6 -1
                //这块不必细究边缘值到底是不是划分的准（例如考虑前五个点是不是都不要，还是说只不要前四个点），
                //只是尽可能的分开成六段，首尾相接的地方不要。因为庞大的点云中，一两个点其实无关紧要。
                int sp = (startRingIndex[i] * (6 - j) + endRingIndex[i] * j) / 6;
                int ep = (startRingIndex[i] * (5 - j) + endRingIndex[i] * (j + 1)) / 6 - 1;

                if (sp >= ep)
                    continue;
                // 按照曲率从小到大排序点云
                //可以看出之前的byvalue在这里被当成了判断函数来用
                std::sort(cloudSmoothness.begin() + sp, cloudSmoothness.begin() + ep, by_value());

                int largestPickedNum = 0;
                // 按照曲率从大到小遍历
                for (int k = ep; k >= sp; k--)
                {
                    // 激光点的索引
                    int ind = cloudSmoothness[k].ind;
                    // 当前激光点还未被处理，且曲率大于阈值，则认为是角点
                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > edgeThreshold)
                    {
                        // 每段只取20个角点，如果单条扫描线扫描一周是1800个点，则划分6段，每段300个点，从中提取20个角点
                        largestPickedNum++;
                        if (largestPickedNum <= 20)
                        {
                            // 标记为角点,加入角点点云
                            cloudLabel[ind] = 1;
                            cornerCloud->push_back(extractedCloud->points[ind]);
                        }
                        else
                        {
                            break;
                        }
                        // 标记已被处理
                        cloudNeighborPicked[ind] = 1;

                        // 同一条扫描线上后5个点标记一下，不再处理，避免特征聚集
                        for (int l = 1; l <= 5; l++)
                        {
                            int columnDiff = std::abs(int(pointColInd[ind + l] - pointColInd[ind + l - 1]));
                            //大于10，说明距离远，则不作标记
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                        // 同一条扫描线上前5个点标记一下，不再处理，避免特征聚集
                        for (int l = -1; l >= -5; l--)
                        {
                            int columnDiff = std::abs(int(pointColInd[ind + l] - pointColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }
                // 按照曲率从小到大遍历
                for (int k = sp; k <= ep; k++)
                {
                    // 激光点的索引
                    int ind = cloudSmoothness[k].ind;
                    // 当前激光点还未被处理，且曲率小于阈值，则认为是平面点
                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < surfThreshold)
                    {
                        // 标记为平面点
                        cloudLabel[ind] = -1;
                        // 标记已被处理
                        cloudNeighborPicked[ind] = 1;
                        // 同一条扫描线上后5个点标记一下，不再处理，避免特征聚集
                        for (int l = 1; l <= 5; l++)
                        {

                            int columnDiff = std::abs(int(pointColInd[ind + l] - pointColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        // 同一条扫描线上前5个点标记一下，不再处理，避免特征聚集
                        for (int l = -1; l >= -5; l--)
                        {

                            int columnDiff = std::abs(int(pointColInd[ind + l] - pointColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }
                // 平面点和未被处理的点(<=0)，都认为是平面点，加入平面点云集合
                for (int k = sp; k <= ep; k++)
                {
                    if (cloudLabel[k] <= 0)
                    {
                        surfaceCloudScan->push_back(extractedCloud->points[k]);
                    }
                }
            }
            // 平面点云降采样
            surfaceCloudScanDS->clear();
            downSizeFilter.setInputCloud(surfaceCloudScan);
            downSizeFilter.filter(*surfaceCloudScanDS);
            // 加入平面点云集合
            *surfaceCloud += *surfaceCloudScanDS;
            //用surfaceCloudScan来装数据，然后放到downSizeFilter里，
            //再用downSizeFilter进行.filter()操作，把结果输出到*surfaceCloudScanDS里。
            //最后把DS装到surfaceCloud中。DS指的是DownSample。
            //同样角点（边缘点）则没有这样的操作，直接就用cornerCloud来装点云。
        }
    }
};
