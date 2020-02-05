#ifndef COMMON_H
#define COMMON_H

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"


#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

using namespace Eigen;

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> CloudT;
typedef CloudT::Ptr CloudTPtr;
typedef CloudT::ConstPtr CloudTConstPtr;

typedef pcl::PointXYZ PointXZY;
typedef pcl::PointCloud<PointXZY> CloudXYZ;
typedef CloudXYZ::Ptr CloudXYZPtr;
typedef CloudXYZ::ConstPtr CloudXYZConstPtr;

typedef Eigen::Matrix<double, 3, 1> Vec3;
typedef Eigen::Matrix<double, 4, 1> Vec4;
typedef Eigen::Matrix<double, 6, 1> Vec6;
typedef Eigen::Matrix<double, 7, 1> Vec7;
typedef Eigen::Matrix<double, 9, 1> Vec9;
typedef Eigen::Matrix<double, 12, 1> Vec12;
typedef Eigen::Matrix<double, 15, 1> Vec15;
typedef Eigen::Matrix<double, 16, 1> Vec16;

typedef Eigen::Matrix<double, 1, 1> Mat1x1;
typedef Eigen::Matrix<double, 3, 3> Mat3x3;
typedef Eigen::Matrix<double, 4, 4> Mat4x4;
typedef Eigen::Matrix<double, 6, 6> Mat6x6;
typedef Eigen::Matrix<double, 7, 7> Mat7x7;
typedef Eigen::Matrix<double, 9, 9> Mat9x9;
typedef Eigen::Matrix<double, 12, 12> Mat12x12;
typedef Eigen::Matrix<double, 15, 15> Mat15x15;




typedef Eigen::Matrix<double, 3, 4> Mat3x4;
typedef Eigen::Matrix<double, 4, 3> Mat4x3;
typedef Eigen::Matrix<double, 15, 6> Mat15x6;
typedef Eigen::Matrix<double, 6, 15> Mat6x15;
typedef Eigen::Matrix<double, 9, 15> Mat9x15;
typedef Eigen::Matrix<double, 15, 12> Mat15x12;
typedef Eigen::Matrix<double, 15, 9> Mat15x9;
typedef Eigen::Matrix<double, 3, 15> Mat3x15;
typedef Eigen::Matrix<double, 15, 3> Mat15x3;
typedef Eigen::Matrix<double, 7, 15> Mat7x15;
typedef Eigen::Matrix<double, 15, 7> Mat15x7;
typedef Eigen::Matrix<double, 7, 16> Mat7x16;
typedef Eigen::Matrix<double, 16, 7> Mat16x7;

typedef Eigen::Matrix<double, 1, 15> Mat1x15;
typedef Eigen::Matrix<double, 15, 1> Mat15x1;

struct IMU_DATA
{
    double tstamp;
    double dT;//time between this fram and last frame
    Quaterniond q;
    Vector3d    acc_bf;//The data is in body frame
    Vector3d    gyro_bf;
};


#endif // COMMON_H
