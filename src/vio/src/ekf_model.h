#ifndef KEF_MODEL_H
#define KEF_MODEL_H

#include <eigen3/Eigen/Dense>
#include <common.h>


Vec15 xdot(const Vec15& x, const Vec6& u, const Vec12& n);
Mat15x15 jacobian_A(const Vec15& x, const Vec6& u, const Vec12& n);
Mat15x12 jacobian_U(const Vec15& x, const Vec6& u, const Vec12& n);
Mat6x15  jacobian_C_vo(const Vec15& x);//observation matrix of state
Mat6x6   jacobian_W_vo(const Vec15& x, const Vec6& v);//observation matrix of observation noise
Vec3     velocity_obsrvation_in_camera(const Vec15& x, const Vec3& v);
#endif // KEF_MODEL_H
