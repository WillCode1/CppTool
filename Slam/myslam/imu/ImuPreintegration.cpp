#include "utility.h"
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

namespace imu
{
    using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
    using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
    using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

    class ImuPreintegration
    {
    public:
        bool systemInitialized = false;
        std::mutex mtx;

        // 噪声协方差
        gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
        gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
        gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
        gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
        gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;
        gtsam::Vector noiseModelBetweenBias;

        // imu预积分器
        // imuIntegratorOpt_负责预积分两个激光里程计之间的imu数据，作为约束加入因子图，并且优化出bias
        gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
        // imuIntegratorImu_用来根据新的激光里程计到达后已经优化好的bias，预测从当前帧开始，下一帧激光里程计到达之前的imu里程计增量
        gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;

        // imu数据队列
        // imuQueOpt用来给imuIntegratorOpt_提供数据来源，不要的就弹出(从队头开始出发，比当前激光里程计数据早的imu通通积分，用一个扔一个)；
        std::deque<ImuData> imuQueOpt;
        // imuQueImu用来给imuIntegratorImu_提供数据来源，不要的就弹出(弹出当前激光里程计之前的imu数据,预积分用完一个弹一个)；
        std::deque<ImuData> imuQueImu;

        // imu因子图优化过程中的状态变量
        gtsam::Pose3 prevPose_;
        gtsam::Vector3 prevVel_;
        gtsam::NavState prevState_;
        gtsam::imuBias::ConstantBias prevBias_;

        // imu状态
        gtsam::NavState prevStateOdom;
        gtsam::imuBias::ConstantBias prevBiasOdom;

        bool doneFirstOpt = false;
        double lastImuT_imu = -1;
        double lastImuT_opt = -1;

        // ISAM2优化器
        gtsam::ISAM2 optimizer;
        gtsam::NonlinearFactorGraph graphFactors; //总的因子图模型
        gtsam::Values graphValues;                //因子图模型中的值

        const double delta_t = 0;

        int key = 1;

        // imu-lidar位姿变换
        //这点要注意，tixiaoshan这里命名的很垃圾，这只是一个平移变换，
        //同样头文件的imuConverter中，也只有一个旋转变换。这里绝对不可以理解为把imu数据转到lidar下的变换矩阵。
        //事实上，作者后续是把imu数据先用imuConverter旋转到雷达系下（但其实还差了个平移）。
        //作者真正是把雷达数据又根据lidar2Imu反向平移了一下，和转换以后差了个平移的imu数据在“中间系”对齐，
        //之后算完又从中间系通过imu2Lidar挪回了雷达系进行publish。
        gtsam::Pose3 imu2Lidar;
        gtsam::Pose3 lidar2Imu;

        Eigen::Matrix3d extrinsicRot;     // extrinsicRot
        Eigen::Quaterniond extrinsicQRPY; // extrinsicRPY

        ImuPreintegration(float imuAccNoise, float imuGyrNoise, float imuAccBiasN, float imuGyrBiasN, float imuGravity,
                          const Eigen::Matrix3d &extRot, const Eigen::Matrix3d &extRPY, const Eigen::Vector3d &extrinsicTrans)
        {
            extrinsicRot = extRot;
#if 1
            extrinsicRot << -1, 0, 0,
                0, 1, 0,
                0, 0, -1;

            // extRPY << 0, 1, 0,
            //     -1, 0, 0,
            //     0, 0, 1;

            // extrinsicTrans = Eigen::Vector3d(0, 0, 0);

            imuAccNoise = 0.01;
            imuGyrNoise = 0.001;
            imuAccBiasN = 0.0002;
            imuGyrBiasN = 0.00003;
            imuGravity = 9.80511;
#endif
            extrinsicQRPY = Eigen::Quaterniond(extRPY);

            imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-extrinsicTrans.x(), -extrinsicTrans.y(), -extrinsicTrans.z()));
            lidar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(extrinsicTrans.x(), extrinsicTrans.y(), extrinsicTrans.z()));

            // imu预积分的噪声协方差
            boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);

            // imuAccNoise和imuGyrNoise都是定义在头文件中的高斯白噪声，由配置文件中写入
            p->accelerometerCovariance = gtsam::Matrix33::Identity(3, 3) * pow(imuAccNoise, 2); // acc white noise in continuous
            p->gyroscopeCovariance = gtsam::Matrix33::Identity(3, 3) * pow(imuGyrNoise, 2);     // gyro white noise in continuous
            //对于速度的积分误差？这块暂时不太理解
            p->integrationCovariance = gtsam::Matrix33::Identity(3, 3) * pow(1e-4, 2); // error committed in integrating position from velocities
            //假设没有初始的bias
            gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished()); // assume zero initial bias

            // 噪声先验
            // Diagonal对角线矩阵
            //发现diagonal型一般调用.finished(),注释中说finished()意为设置完所有系数后返回构建的矩阵
            priorPoseNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
            priorVelNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e4);                                                               // m/s
            priorBiasNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);                                                             // 1e-2 ~ 1e-3 seems to be good
            // 激光里程计scan-to-map优化过程中发生退化，则选择一个较大的协方差
            correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished()); // rad,rad,rad,m, m, m
            correctionNoise2 = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished());               // rad,rad,rad,m, m, m
            noiseModelBetweenBias = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();

            // imu预积分器，用于预测每一时刻（imu频率）的imu里程计（转到lidar系了，与激光里程计同一个系）
            imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for IMU message thread
            // imu预积分器，用于因子图优化
            imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for optimization
        }

        void resetOptimization()
        {
            gtsam::ISAM2Params optParameters;
            optParameters.relinearizeThreshold = 0.1;
            optParameters.relinearizeSkip = 1;
            optimizer = gtsam::ISAM2(optParameters);

            gtsam::NonlinearFactorGraph newGraphFactors;
            graphFactors = newGraphFactors;

            gtsam::Values NewGraphValues;
            graphValues = NewGraphValues;
        }

        void resetParams()
        {
            lastImuT_imu = -1;
            doneFirstOpt = false;
            systemInitialized = false;
        }

        // 订阅的是激光里程计,"lio_sam/mapping/odometry_incremental"
        // bool degenerate = (int)odomMsg->pose.covariance[0] == 1 ? true : false;
        void odometryHandler(const OdometryData &odomMsg, bool degenerate)
        {
            std::lock_guard<std::mutex> lock(mtx);
            // 当前帧激光里程计时间戳
            double currentCorrectionTime = odomMsg.stamp;

            // 确保imu优化队列中有imu数据进行预积分
            if (imuQueOpt.empty())
                return;

            // 当前帧激光位姿，来自scan-to-map匹配、因子图优化后的位姿
            float p_x = odomMsg.pose.position.x();
            float p_y = odomMsg.pose.position.y();
            float p_z = odomMsg.pose.position.z();
            float r_x = odomMsg.pose.orientation.x();
            float r_y = odomMsg.pose.orientation.y();
            float r_z = odomMsg.pose.orientation.z();
            float r_w = odomMsg.pose.orientation.w();
            gtsam::Pose3 lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));

            // 0. initialize system
            // 0. 系统初始化，第一帧
            if (!systemInitialized)
            {
                // 重置ISAM2优化器
                resetOptimization();

                // pop old IMU message
                // 从imu优化队列中删除当前帧激光里程计时刻之前的imu数据,delta_t=0
                while (!imuQueOpt.empty())
                {
                    if (imuQueOpt.front().stamp < currentCorrectionTime - delta_t)
                    {
                        lastImuT_opt = imuQueOpt.front().stamp;
                        imuQueOpt.pop_front();
                    }
                    else
                        break;
                }
                // initial pose
                // 添加里程计位姿先验因子
                // lidarPose 为本回调函数收到的激光里程计数据，重组成gtsam的pose格式
                //并转到imu坐标系下,我猜测compose可能类似于左乘之类的含义吧
                prevPose_ = lidarPose.compose(lidar2Imu);
                // X可能是固定搭配（当使用Pose时），如果是速度则是V，bias则是B
                gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
                //通过调用总的因子图模型的add方式，添加第一个因子
                // PriorFactor 概念可看gtsam  包括了位姿 速度  bias
                //加入PriorFactor在图优化中基本都是必需的前提
                //各种noise都定义在构造函数当中
                graphFactors.add(priorPose);
                // initial velocity
                prevVel_ = gtsam::Vector3(0, 0, 0);
                gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
                graphFactors.add(priorVel);
                // initial bias
                prevBias_ = gtsam::imuBias::ConstantBias();
                gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
                graphFactors.add(priorBias);
                // add values
                // 变量节点赋初值
                graphValues.insert(X(0), prevPose_);
                graphValues.insert(V(0), prevVel_);
                graphValues.insert(B(0), prevBias_);
                // optimize once
                // 优化一次
                optimizer.update(graphFactors, graphValues);
                //图和节点均清零  为什么要清零不能继续用吗?
                //是因为节点信息保存在gtsam::ISAM2 optimizer，所以要清理后才能继续使用
                graphFactors.resize(0);
                graphValues.clear();

                //积分器重置,重置优化之后的偏置
                imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
                imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

                key = 1;
                systemInitialized = true;
                return;
            }

            // reset graph for speed
            // 每隔100帧激光里程计，重置ISAM2优化器，保证优化效率
            if (key == 100)
            {
                // get updated noise before reset
                // 前一帧的位姿、速度、偏置噪声模型
                //保存最后的噪声值
                gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key - 1)));
                gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(V(key - 1)));
                gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(B(key - 1)));
                // reset graph
                // 重置ISAM2优化器
                resetOptimization();
                // add pose
                // 添加位姿先验因子，用前一帧的值初始化
                //重置之后还有类似与初始化的过程 区别在于噪声值不同
                // prevPose_等三项，也是上一时刻得到的，
                //（初始时刻是lidar里程计的pose直接用lidar2IMU变量转到imu坐标系下，而此处则是通过上一时刻，即接下来的后续优化中得到）
                gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
                graphFactors.add(priorPose);
                // add velocity
                gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, updatedVelNoise);
                graphFactors.add(priorVel);
                // add bias
                gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, updatedBiasNoise);
                graphFactors.add(priorBias);
                // add values
                graphValues.insert(X(0), prevPose_);
                graphValues.insert(V(0), prevVel_);
                graphValues.insert(B(0), prevBias_);
                // optimize once
                optimizer.update(graphFactors, graphValues);
                graphFactors.resize(0);
                graphValues.clear();

                key = 1;
            }

            // 1. integrate imu data and optimize
            // 1. 计算前一帧与当前帧之间的imu预积分量，用前一帧状态施加预积分量得到当前帧初始状态估计，
            //  添加来自mapOptimization的当前帧位姿，进行因子图优化，更新当前帧状态
            while (!imuQueOpt.empty())
            {
                // pop and integrate imu data that is between two optimizations
                // 提取前一帧与当前帧之间的imu数据，计算预积分
                ImuData *thisImu = &imuQueOpt.front();
                double imuTime = thisImu->stamp;
                // currentCorrectionTime是当前回调函数收到的激光里程计数据的时间
                if (imuTime < currentCorrectionTime - delta_t)
                {
                    double dt = (lastImuT_opt < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_opt);
                    // imu预积分数据输入：加速度、角速度、dt
                    // 加入的是这个用来因子图优化的预积分器imuIntegratorOpt_,注意加入了上一步算出的dt
                    //作者要求的9轴imu数据中欧拉角在本程序文件中没有任何用到,全在地图优化里用到的
                    imuIntegratorOpt_->integrateMeasurement(
                        gtsam::Vector3(thisImu->linear_acceleration.x(), thisImu->linear_acceleration.y(), thisImu->linear_acceleration.z()),
                        gtsam::Vector3(thisImu->angular_velocity.x(), thisImu->angular_velocity.y(), thisImu->angular_velocity.z()), dt);
                    //在推出一次数据前保存上一个数据的时间戳

                    lastImuT_opt = imuTime;
                    // 从队列中删除已经处理的imu数据
                    imuQueOpt.pop_front();
                }
                else
                    break;
            }
            // add imu factor to graph
            //利用两帧之间的IMU数据完成了预积分后增加imu因子到因子图中,
            //注意后面容易被遮挡，imuIntegratorOpt_的值经过格式转换被传入preint_imu，
            //因此可以推测imuIntegratorOpt_中的integrateMeasurement函数应该就是一个简单的积分轮子，
            //传入数据和dt，得到一个积分量,数据会被存放在imuIntegratorOpt_中
            const auto &preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements &>(*imuIntegratorOpt_);
            // 参数：前一帧位姿，前一帧速度，当前帧位姿，当前帧速度，前一帧偏置，预计分量
            gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
            graphFactors.add(imu_factor);
            // add imu bias between factor
            // 添加imu偏置因子，前一帧偏置B(key - 1)，当前帧偏置B(key)，观测值，噪声协方差；deltaTij()是积分段的时间
            graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                                                                                gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));
            // add pose factor
            // 添加位姿因子
            gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);
            gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, degenerate ? correctionNoise2 : correctionNoise);
            graphFactors.add(pose_factor);
            // insert predicted values
            // 用前一帧的状态、偏置，施加imu预计分量，得到当前帧的状态
            gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
            // 变量节点赋初值
            graphValues.insert(X(key), propState_.pose());
            graphValues.insert(V(key), propState_.v());
            graphValues.insert(B(key), prevBias_);
            // optimize
            optimizer.update(graphFactors, graphValues);
            optimizer.update();
            graphFactors.resize(0);
            graphValues.clear();
            // Overwrite the beginning of the preintegration for the next step.
            // 优化结果
            gtsam::Values result = optimizer.calculateEstimate();
            // 更新当前帧位姿、速度
            prevPose_ = result.at<gtsam::Pose3>(X(key));
            prevVel_ = result.at<gtsam::Vector3>(V(key));
            // 更新当前帧状态
            prevState_ = gtsam::NavState(prevPose_, prevVel_);
            // 更新当前帧imu偏置
            prevBias_ = result.at<gtsam::imuBias::ConstantBias>(B(key));
            // Reset the optimization preintegration object.
            //重置预积分器，设置新的偏置，这样下一帧激光里程计进来的时候，预积分量就是两帧之间的增量
            imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
            // check optimization
            // imu因子图优化结果，速度或者偏置过大，认为失败
            if (failureDetection(prevVel_, prevBias_))
            {
                resetParams();
                return;
            }

            // 2. after optiization, re-propagate imu odometry preintegration
            // 2. 优化之后，执行重传播；优化更新了imu的偏置，
            //用最新的偏置重新计算当前激光里程计时刻之后的imu预积分，这个预积分用于计算每时刻位姿
            prevStateOdom = prevState_;
            prevBiasOdom = prevBias_;
            // first pop imu message older than current correction data
            // 从imu队列中删除当前激光里程计时刻之前的imu数据
            double lastImuQT = -1;
            //注意，这里是要“删除”当前帧“之前”的imu数据，是想根据当前帧“之后”的累积递推。
            //而前面imuIntegratorOpt_做的事情是，“提取”当前帧“之前”的imu数据，用两帧之间的imu数据进行积分。处理过的就弹出来。
            //因此，新到一帧激光帧里程计数据时，imuQueOpt队列变化如下：
            //当前帧之前的数据被提出来做积分，用一个删一个（这样下一帧到达后，队列中就不会有现在这帧之前的数据了）
            //那么在更新完以后，imuQueOpt队列不再变化，剩下的原始imu数据用作下一次优化时的数据。
            //而imuQueImu队列则是把当前帧之前的imu数据都给直接剔除掉，仅保留当前帧之后的imu数据，
            //用作两帧lidar里程计到达时刻之间发布的imu增量式里程计的预测。
            // imuQueImu和imuQueOpt的区别要明确,imuIntegratorImu_和imuIntegratorOpt_的区别也要明确,见imuhandler中的注释

            while (!imuQueImu.empty() && imuQueImu.front().stamp < currentCorrectionTime - delta_t)
            {
                lastImuQT = imuQueImu.front().stamp;
                imuQueImu.pop_front();
            }
            // repropogate
            // 对剩余的imu数据计算预积分
            if (!imuQueImu.empty())
            {
                // reset bias use the newly optimized bias
                // 传入状态,重置预积分器和最新的偏置
                imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
                // integrate imu message from the beginning of this optimization
                // 计算预积分
                //利用imuQueImu中的数据进行预积分 主要区别旧在于上一行的更新了bias
                for (auto &thisImu : imuQueImu)
                {
                    double imuTime = thisImu.stamp;
                    double dt = (lastImuQT < 0) ? (1.0 / 500.0) : (imuTime - lastImuQT);
                    // 注意:加入的是这个用于传播的的预积分器imuIntegratorImu_,(之前用来计算的是imuIntegratorOpt_,）
                    //注意加入了上一步算出的dt
                    //结果被存放在imuIntegratorImu_中
                    imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu.linear_acceleration.x(), thisImu.linear_acceleration.y(), thisImu.linear_acceleration.z()),
                                                            gtsam::Vector3(thisImu.angular_velocity.x(), thisImu.angular_velocity.y(), thisImu.angular_velocity.z()), dt);
                    lastImuQT = imuTime;
                }
            }

            ++key;
            //设置成True，用来通知另一个负责发布imu里程计的回调函数imuHandler“可以发布了”
            doneFirstOpt = true;
        }

        /**
         * imu因子图优化结果，速度或者偏置过大，认为失败
         */
        bool failureDetection(const gtsam::Vector3 &velCur, const gtsam::imuBias::ConstantBias &biasCur)
        {
            Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
            if (vel.norm() > 30)
            {
                std::cout << "Large velocity, reset IMU-preintegration!" << std::endl;
                return true;
            }

            Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
            Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
            if (ba.norm() > 1.0 || bg.norm() > 1.0)
            {
                std::cout << "Large bias, reset IMU-preintegration!" << std::endl;
                return true;
            }

            return false;
        }

        /**
         * 订阅imu原始数据
         * 1、用上一帧激光里程计时刻对应的状态、偏置，施加从该时刻开始到当前时刻的imu预计分量，得到当前时刻的状态，也就是imu里程计
         * 2、imu里程计位姿转到lidar系，发布里程计
         */
        void imuHandler(const ImuData &imu_raw)
        {
            std::lock_guard<std::mutex> lock(mtx);
            // imu原始测量数据转换到lidar系，加速度、角速度、RPY
            ImuData thisImu = Tool::imuConverter(imu_raw, extrinsicRot, extrinsicQRPY);

            // 添加当前帧imu数据到队列
            // 两个双端队列分别装着优化前后的imu数据
            imuQueOpt.push_back(thisImu);
            imuQueImu.push_back(thisImu);

            // 要求上一次imu因子图优化执行成功，确保更新了上一帧（激光里程计帧）的状态、偏置，预积分已经被重新计算
            // 这里需要先在odomhandler中优化一次后再进行该函数后续的工作
            if (!doneFirstOpt)
                return;

            double imuTime = thisImu.stamp;
            // lastImuT_imu变量初始被赋值为-1
            //  获得时间间隔, 第一次为1/500,之后是两次imuTime间的差
            double dt = (lastImuT_imu < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_imu);
            lastImuT_imu = imuTime;

            // integrate this single imu message
            // imu预积分器添加一帧imu数据，注：这个预积分器的起始时刻是上一帧激光里程计时刻
            imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu.linear_acceleration.x(), thisImu.linear_acceleration.y(), thisImu.linear_acceleration.z()),
                                                    gtsam::Vector3(thisImu.angular_velocity.x(), thisImu.angular_velocity.y(), thisImu.angular_velocity.z()), dt);

            // predict odometry
            // 用上一帧激光里程计时刻对应的状态、偏置，施加从该时刻开始到当前时刻的imu预计分量，得到当前时刻的状态
            gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);

            // publish odometry
            // 发布imu里程计（转到lidar系，与激光里程计同一个系）
            OdometryData odometry;

            // transform imu pose to ldiar
            //预测值currentState获得imu位姿, 再由imu到雷达变换, 获得雷达位姿
            gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
            gtsam::Pose3 lidarPose = imuPose.compose(imu2Lidar);
            // 这里留疑问，本cpp读完后补充：
            // 为什么currentState获得的是imu的位姿？原始imu数据难道不是先转换成雷达坐标系下的数据（this imu）才再送到imu预积分器中吗？
            //答： 在之前的优化函数odometryHandler中，thisIMU是直接从imuQueOpt中取值.
            //而imuQueOpt中的内容，是已经从imu原始测量数据转换到了lidar"中间系"系下（在本函数第二行）。离真正的雷达系还差了一个平移
            // odometryHandler函数中根据prevPose_ = lidarPose.compose(lidar2Imu)得到激光帧先验位姿(lidarPose)转换到imu系下,（相当于从真正的雷达系扭到上面的中间系中）
            //作为初值构建了因子进行优化；
            //在其imuIntegratorOpt_->integrateMeasurement中得到的应该是dt之间的预积分量，
            //由于其处在循环中，因此是在递推累积计算两帧之间的预积分量。
            //（相当于是每到一帧，就把二者之间的预积分量计算一遍，并且优化一遍，存储进imuIntegratorOpt_）中。

            //因为本函数为imu回调函数，每收到一个imu数据，当之前因子图算完的情况下，
            //在imuIntegratorImu_的基础上继续递推新收到的imu数据，并且进行预测。
            //最后把imu再转回lidar系下进行发布。
            //注意：这里发布的是两帧之间的“增量”imu里程计信息，
            // imuIntegratorImu_本身是个积分器，只有两帧之间的预积分，但是发布的时候发布的实际是结合了前述里程计本身有的位姿
            //如这个predict里的prevStateOdom:
            // currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);

            //关于imuIntegratorImu_在两个回调函数中都进行integrateMeasurement操作，之间是否会有冲突呢？
            //我觉得关键在于odometryHandler中有一句：imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom)，
            //在imuIntegratorOpt_优化完两帧imu数据之后，imuIntegratorImu_直接把积分和bias给reset掉，
            //然后开始根据imuIntegratorOpt_优化出的bias来更新imuIntegratorImu_。

            // imuIntegratorImu_和imuIntegratorOpt_的区别在于，opt存储的是新到一帧，和上一帧之间的预积分量，作为约束，执行优化。
            //优化后，imuIntegratorImu_利用新的bias，在新到的这一帧的基础上，递推“之后”的预积分量。
            //（绝对不能把imuIntegratorOpt_和imuIntegratorImu_理解为同一批imu数据在优化前后的不同值）

            //在更新的过程中不用担心会被imuHandler中的imuIntegratorImu_->integrateMeasurement给顶掉，
            //这是因为imuHandler要根据doneFirstOpt来检查odometryHandler是不是已经更新完bias了。
            //因为更新并不是实时的，而是一帧激光数据到了才更新。
            //可是下一帧激光并没有到，但是在此期间imu增量式里程计还是得照常发布，
            //如果当前帧已经更新完bias了，然后就可以直接利用这个bias计算后面新到的ImuIntegratorImu_，

            odometry.pose.position.x() = lidarPose.translation().x();
            odometry.pose.position.y() = lidarPose.translation().y();
            odometry.pose.position.z() = lidarPose.translation().z();
            odometry.pose.orientation.x() = lidarPose.rotation().toQuaternion().x();
            odometry.pose.orientation.y() = lidarPose.rotation().toQuaternion().y();
            odometry.pose.orientation.z() = lidarPose.rotation().toQuaternion().z();
            odometry.pose.orientation.w() = lidarPose.rotation().toQuaternion().w();

            odometry.twist.linear.x() = currentState.velocity().x();
            odometry.twist.linear.y() = currentState.velocity().y();
            odometry.twist.linear.z() = currentState.velocity().z();
            odometry.twist.angular.x() = thisImu.angular_velocity.x() + prevBiasOdom.gyroscope().x();
            odometry.twist.angular.y() = thisImu.angular_velocity.y() + prevBiasOdom.gyroscope().y();
            odometry.twist.angular.z() = thisImu.angular_velocity.z() + prevBiasOdom.gyroscope().z();
            // pubImuOdometry.publish(odometry);
        }
    };
    //还有两个问题：
    // 1.第一个是，为什么imu原始数据先要根据imuConverter变到lidar系，
    //那么之后imuintegrator->integrateMeasurement算到的预积分数据不就是lidar系下的吗？
    //在处理的时候又是把lidar里程计的坐标系根据compose函数变到imu系？这难道不是不对应了吗:

    // 2.变量gtsam::Pose3 imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0),
    // gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));
    //这里为什么不用配置文件中的extRot，extQRPY之类的内容呢，而只是用了extTrans数据？

    //关于这点，我在github中找到了解释:https://github.com/TixiaoShan/LIO-SAM/issues/30,
    // imuConverter() to align the axis of the two coordinates,并没有涉及到平移,
    // lidar2Imu or imu2Lidar 却只有平移的内容
    //因此收到imu后，先用imuConverter()转换到雷达系下，（但其实和雷达之间仍然差了一个平移），
    //因此又把雷达的内容用只含有平移的lidar2Imu 和原本差了一个平移的imu数据真正对齐
    //（相当于是imu旋转到雷达系下以后不平移，然后把雷达倒着平移过来,在一个“中间系”对齐）。
    //在算完以后，等发布的时候，又用imu2Lidar又倒回到了正儿八经的雷达系。

    //那么tixiaoshan为什么在默认里把平移参数设置为0，0，0？
    //他在github中的解释为: 我在不同的数据集中改变了几次IMU的安装位置。但是位置总是靠近激光雷达。
    //所以每次测试不同的数据集时，我都不必费心去修改这个参数。严格地说，我的方法并不理想。需要提供此参数以获得更好的性能。
}
