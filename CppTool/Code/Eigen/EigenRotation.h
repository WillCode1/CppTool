#pragma once
#include "Eigen/Dense"
using namespace Eigen;

namespace Eigen_Tool
{
    class EigenRotation
    {
    public:
        // 一、旋转向量
        // 1.1 旋转向量转旋转矩阵
        static Eigen::Matrix3d AngleAxis2RotationMatrix(const Eigen::AngleAxisd& angle_axis)
        {
            // 1.0 初始化旋转向量：旋转角为alpha，旋转轴为(x, y, z)
            //Eigen::AngleAxisd rotation_vector(alpha, Vector3d(x, y, z))
            return angle_axis.matrix();
        }

        // 1.2 旋转向量转欧拉角(Z - Y - X，即RPY)
        static Eigen::Vector3d AngleAxis2EulerAngles(const Eigen::AngleAxisd& angle_axis)
        {
            return angle_axis.matrix().eulerAngles(2, 1, 0);
        }
        
        // 1.3 旋转向量转四元数
        static Eigen::Quaterniond AngleAxis2Quaternion(const Eigen::AngleAxisd& angle_axis)
        {
            return Eigen::Quaterniond(angle_axis);
        }

        // 二、旋转矩阵
        // 2.1 旋转矩阵转旋转向量
        static Eigen::AngleAxisd RotationMatrix2AngleAxis(const Eigen::Matrix3d& rotation_matrix)
        {
            // 2.0 初始化旋转矩阵
            // Eigen::Matrix3d rotation_matrix;
            // rotation_matrix << x_00, x_01, x_02, x_10, x_11, x_12, x_20, x_21, x_22;
            return Eigen::AngleAxisd().fromRotationMatrix(rotation_matrix);
        }

        // 2.2 旋转矩阵转欧拉角(Z - Y - X，即RPY)
        static Eigen::Vector3d RotationMatrix2EulerAngles(const Eigen::Matrix3d& rotation_matrix)
        {
            return rotation_matrix.eulerAngles(2, 1, 0);
        }
        
        // 2.3 旋转矩阵转四元数
        static Eigen::Quaterniond RotationMatrix2Quaternion(const Eigen::Matrix3d& rotation_matrix)
        {
            return Eigen::Quaterniond(rotation_matrix);
        }

        // 三、欧拉角
        // 3.1 欧拉角转旋转向量
        static Eigen::AngleAxisd EulerAngles2AngleAxis(const Eigen::Vector3d& eulerAngles)
        {
            // 3.0 初始化欧拉角(Z - Y - X，即RPY)
            // Eigen::Vector3d eulerAngle(yaw, pitch, roll);
            Eigen::AngleAxisd rollAngle(AngleAxisd(eulerAngles(2), Vector3d::UnitX()));
            Eigen::AngleAxisd pitchAngle(AngleAxisd(eulerAngles(1), Vector3d::UnitY()));
            Eigen::AngleAxisd yawAngle(AngleAxisd(eulerAngles(0), Vector3d::UnitZ()));
            Eigen::AngleAxisd angle_axis;
            angle_axis = yawAngle * pitchAngle * rollAngle;
            return angle_axis;
        }

        // 3.2 欧拉角转旋转矩阵
        static Eigen::Matrix3d EulerAngles2RotationMatrix(const Eigen::Vector3d& eulerAngles)
        {
            Eigen::AngleAxisd rollAngle(AngleAxisd(eulerAngles(2), Vector3d::UnitX()));
            Eigen::AngleAxisd pitchAngle(AngleAxisd(eulerAngles(1), Vector3d::UnitY()));
            Eigen::AngleAxisd yawAngle(AngleAxisd(eulerAngles(0), Vector3d::UnitZ()));
            Eigen::Matrix3d rotation_matrix;
            rotation_matrix = yawAngle * pitchAngle * rollAngle;
            return rotation_matrix;
        }

        // 3.3 欧拉角转四元数
        static Eigen::Quaterniond EulerAngles2Quaternion(const Eigen::Vector3d& eulerAngles)
        {
            Eigen::AngleAxisd rollAngle(AngleAxisd(eulerAngles(2), Vector3d::UnitX()));
            Eigen::AngleAxisd pitchAngle(AngleAxisd(eulerAngles(1), Vector3d::UnitY()));
            Eigen::AngleAxisd yawAngle(AngleAxisd(eulerAngles(0), Vector3d::UnitZ()));
            Eigen::Quaterniond quaternion;
            quaternion = yawAngle * pitchAngle * rollAngle;
            return quaternion;
        }

        // 四、四元数
        // 4.1 四元数转旋转向量
        static Eigen::AngleAxisd Quaternion2AngleAxis(const Eigen::Quaterniond& quaternion)
        {
            return Eigen::AngleAxisd(quaternion);
        }

        // 4.2 四元数转旋转矩阵
        static Eigen::Matrix3d Quaternion2RotationMatrix(const Eigen::Quaterniond& quaternion)
        {
            return quaternion.matrix();
        }

        // 4.3 四元数转欧拉角(Z - Y - X，即RPY)
        static Eigen::Vector3d Quaternion2EulerAngles(const Eigen::Quaterniond& quaternion)
        {
            return quaternion.matrix().eulerAngles(2, 1, 0);
        }
    };
}
