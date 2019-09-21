#include <eskf_model.h>
#include <common.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <Eigen/Dense>
#include <iostream>
#include <euler_q_rmatrix.h>
#include <kinetic_math.h>
#include <nav_msgs/Odometry.h>

using namespace std;
using namespace Eigen;

IOFormat CleanFmt(3, 0, "|", "\n", "[", "]");

inline Mat3x3 skew_symmetric_matrix(const Vec3 a)
{
    Mat3x3 A;
    A<<     0,     -a(2), -a(1),
            a(2),  0,     -a(0),
            -a(1), a(0),  0;
    return A;
}


Vec6 votoz(const nav_msgs::Odometry::ConstPtr &msg)
{
    Vec6 z;
    Affine3d tf_WI;
    geometry_msgs::Quaternion q = msg->pose.pose.orientation;
    geometry_msgs::Point t = msg->pose.pose.position;
    /* Tag to camera */
    tf_WI.setIdentity();
    tf_WI.translation() = Vector3d(t.x, t.y, t.z);
    tf_WI.linear() = Matrix3d(Quaterniond(q.w, q.x, q.y, q.z));

    Matrix3d r_WI = tf_WI.linear();
    Vector3d t_WI = tf_WI.translation();


    Vector3d euler = euler_from_rotation_matrix(r_WI);
    z << t_WI(0), t_WI(1), t_WI(2), euler(1), euler(1), euler(2);
    return z;
}

//update the nominal state x with u
void nominal_state_pre_integration(Vec16& x, const Vec6 u, const double dT, const Vec3 g_vec)
{
    //x
    //[0 - 3]   qw,qx,qy,qz
    //[4 - 6]   px,py,pz
    //[7 - 9]   vx,vy,vz
    //[10 -12]  bwx,bwy,bwz
    //[13-15]   bax,bay,baz
    Vec4  q=x.head(4);
    Vec3  p=x.segment(3,4);
    Vec3  v=x.segment(3,7);
    Vec3  wb=x.segment(3,10);
    Vec3  ab=x.segment(3,13);
    //u
    //[0 - 2]   wm  //gyro measurement
    //[3 - 5]   am  //acc  measurement
    Vec3 wm=u.head(3);
    Vec3 am=u.tail(3);

    //q
    Vec3 w=wm-wb;
    Mat4x4 int_mat;
    //q_integration_matrix q*w = mat[w]*q where [] follow:
    //  0, -wx, -wy, -wz,
    // wx,   0,  wz, -wy,
    // wy, -wz,   0,  wx,
    // wz,  wy, -wx,   0,
    int_mat<<  0, -w(0), -w(1), -w(2),
            w(0),     0,  w(2), -w(1),
            w(1), -w(2),     0,  w(0),
            w(2),  w(1), -w(0),     0;
    Vec4 q_diff = ((0.5*int_mat)*q)*dT;
    Vec4 q_new = q+q_diff;

    //p
    Mat3x3 R = Quaterniond().toRotationMatrix();
    Vec3 p_diff = dT*v + 0.5*dT*dT*(R*(am-ab)+g_vec);
    Vec3 p_new = p+p_diff;

    //v
    Vec3 v_diff = dT*(R*(am-ab)+g_vec);
    Vec3 v_new= v + v_diff;

    //wb and ab remain same
    x.head(4)=q_new;
    x.segment(3,4)=p_new;
    x.segment(3,7)=v_new;
}

void error_state_update(Vec15& x_error, Mat15x15& COV, //output
                        const Vec16& x, const Vec6 u, const Mat12x12 Q, const double dT)
{
    //   0              3  6    9     12
    //0  R^T(ωm−ωb)∆t | 0 |0   |−I∆t |0
    //3  0            | I |I∆t |0    |0
    //6  −R[am−ab]×∆t | 0 |I   |0    |−R∆t
    //9  0            | 0 |0   |I    |0
    //12 0            | 0 |0   |0    |I
    Eigen::Quaterniond q(x(0),x(1),x(2),x(3));
    Mat3x3 R=q.toRotationMatrix();
    Vec3 wm=u.head(3);
    Vec3 am=u.tail(3);
    Vec3 wb=x.segment(3,10);
    Vec3 ab=x.segment(3,13);

    Mat15x15 F_ES;
    Mat15x12 Fi_ES;
    Mat3x3 I3x3;
    I3x3.setIdentity();
    F_ES.setZero();
    //    F_ES.block<3,3>(0,0)=R*(am-ab);
    //    F_ES.block<3,3>(0,9)=-dT*I3x3;
    //    F_ES.block<3,3>(3,3)=I3x3;
    //    F_ES.block<3,3>(3,6)=dT*I3x3;
    //    F_ES.block<3,3>(6,0)=dT*(-R)*(am-ab);
    //    F_ES.block<3,3>(6,6)=I3x3;
    //    F_ES.block<3,3>(6,12)=dT*(-R);
    //    F_ES.block<3,3>(9,9)=I3x3;
    //    F_ES.block<3,3>(12,12)=I3x3;
}

















