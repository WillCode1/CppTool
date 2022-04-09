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

        ImuPreintegration(float imuAccNoise, float imuGyrNoise, float imuAccBiasN, float imuGyrBiasN, float imuGravity)
        {
            imuAccNoise = 0.01;
            imuGyrNoise = 0.001;
            imuAccBiasN = 0.0002;
            imuGyrBiasN = 0.00003;
            imuGravity = 9.80511;

            InitLaserImuExtrinsicParam();

            boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);

            p->accelerometerCovariance = gtsam::Matrix33::Identity(3, 3) * pow(imuAccNoise, 2); // acc white noise in continuous
            p->gyroscopeCovariance = gtsam::Matrix33::Identity(3, 3) * pow(imuGyrNoise, 2);     // gyro white noise in continuous
            //对于速度的积分误差？这块暂时不太理解
            p->integrationCovariance = gtsam::Matrix33::Identity(3, 3) * pow(1e-4, 2); // error committed in integrating position from velocities
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

        void InitLaserImuExtrinsicParam()
        {
            extrinsicRot << -1, 0, 0,
                0, 1, 0,
                0, 0, -1;

            Eigen::Matrix3d extRPY;
            extRPY << 0, 1, 0,
                -1, 0, 0,
                0, 0, 1;

            Eigen::Vector3d extrinsicTrans = Eigen::Vector3d(0, 0, 0);
            extrinsicQRPY = Eigen::Quaterniond(extRPY);

            imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-extrinsicTrans.x(), -extrinsicTrans.y(), -extrinsicTrans.z()));
            lidar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(extrinsicTrans.x(), extrinsicTrans.y(), extrinsicTrans.z()));
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
            if (!systemInitialized)
            {
                // 重置ISAM2优化器
                resetOptimization();

                // pop old IMU message
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
                // 并转到imu坐标系下,我猜测compose可能类似于左乘之类的含义吧
                prevPose_ = lidarPose.compose(lidar2Imu);
                gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
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
                graphValues.insert(X(0), prevPose_);
                graphValues.insert(V(0), prevVel_);
                graphValues.insert(B(0), prevBias_);
                // optimize once
                optimizer.update(graphFactors, graphValues);
                // 图和节点均清零, 是因为节点信息保存在gtsam::ISAM2 optimizer，所以要清理后才能继续使用
                graphFactors.resize(0);
                graphValues.clear();

                //积分器重置, 重置优化之后的偏置
                imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
                imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

                key = 1;
                systemInitialized = true;
                return;
            }

            // reset graph for optimizer speed
            if (key == 100)
            {
                // get last updated noise before reset
                gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key - 1)));
                gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(V(key - 1)));
                gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(B(key - 1)));
                // reset graph
                resetOptimization();
                // add prior pose by last
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
            while (!imuQueOpt.empty())
            {
                // pop and integrate imu data that is between two optimizations
                ImuData *thisImu = &imuQueOpt.front();
                double imuTime = thisImu->stamp;
                if (imuTime < currentCorrectionTime - delta_t)
                {
                    double dt = (lastImuT_opt < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_opt);
                    imuIntegratorOpt_->integrateMeasurement(
                        gtsam::Vector3(thisImu->linear_acceleration.x(), thisImu->linear_acceleration.y(), thisImu->linear_acceleration.z()),
                        gtsam::Vector3(thisImu->angular_velocity.x(), thisImu->angular_velocity.y(), thisImu->angular_velocity.z()), dt);

                    lastImuT_opt = imuTime;
                    imuQueOpt.pop_front();
                }
                else
                    break;
            }
            // add imu factor to graph
            const auto &preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements &>(*imuIntegratorOpt_);
            // 参数：前一帧位姿，前一帧速度，当前帧位姿，当前帧速度，前一帧偏置，预计分量
            gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
            graphFactors.add(imu_factor);
            // add imu bias between factor
            // 添加imu偏置因子，前一帧偏置B(key - 1)，当前帧偏置B(key)，观测值，噪声协方差；deltaTij()是积分段的时间
            graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                                                                                gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));
            // add pose factor
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

            // 保存下一步预积分时的初始状态
            gtsam::Values result = optimizer.calculateEstimate();
            prevPose_ = result.at<gtsam::Pose3>(X(key));
            prevVel_ = result.at<gtsam::Vector3>(V(key));
            prevState_ = gtsam::NavState(prevPose_, prevVel_);
            prevBias_ = result.at<gtsam::imuBias::ConstantBias>(B(key));

            // Reset the optimization preintegration object.
            //重置预积分器，设置新的偏置，这样下一帧激光里程计进来的时候，预积分量就是两帧之间的增量
            imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
            // check optimization
            if (failureDetection(prevVel_, prevBias_))
            {
                resetParams();
                return;
            }

            // 2. after optiization, re-propagate imu odometry preintegration
            // 2. 优化更新了imu的bias之后，执行重传播
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
            if (!imuQueImu.empty())
            {
                // reset bias use the newly optimized bias
                imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
                // 利用imuQueImu中的数据 和 优化后的bias, 重新传播预积分
                for (auto &thisImu : imuQueImu)
                {
                    double imuTime = thisImu.stamp;
                    double dt = (lastImuQT < 0) ? (1.0 / 500.0) : (imuTime - lastImuQT);
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

            // 确保更新过了一帧（激光里程计帧）的状态、偏置，预积分已经被重新计算
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
            // 预测值currentState获得imu位姿, 再由imu到雷达变换, 获得雷达位姿
            gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
            gtsam::Pose3 lidarPose = imuPose.compose(imu2Lidar);

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
}