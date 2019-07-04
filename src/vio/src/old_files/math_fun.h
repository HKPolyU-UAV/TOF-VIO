#ifndef MATH_FUN_H
#define MATH_FUN_H

#include <eigen3/Eigen/Eigen>
using namespace Eigen;


inline Matrix3d rotation_matrix_from_euler(const Vector3d rpy)
{
    //1->yaw
    //2->roll
    //3->pitch
    Matrix3d R;

    double psi=rpy(2);//psi
    double theta=rpy(1);//theta
    double phi=rpy(0);//phi

    double r11 = cos(psi)*cos(theta)-sin(phi)*sin(psi)*sin(theta);
    double r12 = -cos(phi)*sin(psi);
    double r13 = cos(psi)*sin(theta)+cos(theta)*sin(phi)*sin(psi);

    double r21 = cos(theta)*sin(psi)+cos(psi)*sin(phi)*sin(theta);
    double r22 = cos(phi)*cos(psi);
    double r23 = sin(psi)*sin(theta)-cos(psi)*cos(theta)*sin(phi);

    double r31 = -cos(phi)*sin(theta);
    double r32 = sin(phi);
    double r33 = cos(phi)*cos(theta);

    R << r11 , r12 , r13 , r21 , r22 , r23 , r31 , r32 , r33;

    return R;
}


inline Vector3d euler_from_rotation_matrix(const Matrix3d R)
{
    Vector3d rpy;
    double roll, pitch, yaw;
    double r12 = R(0,1);
    double r22 = R(1,1);
    double r31 = R(2,0);
    double r32 = R(2,1);
    double r33 = R(2,2);

    roll  = asin(r32);
//    pitch = atan2(-r31/cos(roll),r33/cos(roll));
//    yaw   = atan2(-r12/cos(roll),r22/cos(roll));

    pitch = atan2(-r31,r33);
    yaw   = atan2(-r12,r22);

    rpy(0) = roll;
    rpy(1) = pitch;
    rpy(2) = yaw;

    return rpy;
}
#endif // MATH_FUN_H

