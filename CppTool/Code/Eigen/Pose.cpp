#include "Eigen/Dense"
#include <iostream>
#include "Pose.h"
using namespace std;
using namespace Eigen;

using Pose2d = Eigen::Isometry2d;
using Pose3d = Eigen::Isometry3d;


//https://blog.csdn.net/xhtchina/article/details/111489117
//https://blog.csdn.net/wangxiao7474/article/details/103612669?spm=1001.2014.3001.5501
//http://t.zoukankan.com/winslam-p-12765822.html

void testPose3d()
{
    Pose3d pose;
    pose.setIdentity();

    // 按照q1rotation_matrix进行旋转
    Quaterniond quaternion(1, 0, 0, 0);
    pose.rotate(quaternion);

    // 把平移向量设成t
    Vector3d t(3, 4, 5);
    /* 
        pretranslate(t)表示对三维空间向量v进行旋转操作R后，直接加上该平移量t，即Rv+t；
        而translate(t)需要对t也进行旋转操作，即Rv+Rt。
    */
    pose.pretranslate(t);

    cout << pose.matrix() << endl;
    cout << pose.translation() << endl;
}
