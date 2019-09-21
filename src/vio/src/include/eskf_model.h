#ifndef VIO_EKF_FUSION_H
#define VIO_EKF_FUSION_H

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <common.h>
#include <deque>




//when vision state comming in
void vo_measurement_update();
void re_integraion();
//injection to nominal state

void nominal_state_pre_integration(Vec16& x, const Vec6 u, const double dT, const Vec3 g_vec);
void error_state_update(Vec15& x_error, Mat15x15& COV, const Vec16& x, const Vec6 u, const Mat12x12 Q, const double dT);

#endif // VIO_EKF_FUSION_H
