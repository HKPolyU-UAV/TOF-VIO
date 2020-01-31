#include "ekf_model.h"

Vec15 xdot(const Vec15& x, const Vec6& u, const Vec12& n)
{
    Vec15 xdot;

    //  1: x0:2 ~ x, y, z
    //  2: x3:5 ~ phi theta psi
    //  3: x6:8 ~ vx vy vz
    //  4: x9:11 ~ bgx bgy bgz
    //  5: x12:14 ~  bax bay baz

    // u0:2 wmx, wmy, wmz
    // u3:5 amx, amy, amz

    // n0:2 ngx, ngy, ngz
    // n3:5 nax, nay, naz
    // n6:8 nbgx, nbgy, nbgz
    // n9:11 nbax, nbay, nbaz

    /* ---------- 1 ---------- */
    xdot[0] = x[6];
    xdot[1] = x[7];
    xdot[2] = x[8];

    /* ---------- 2 ---------- */
    double r = x[3];
    double p = x[4];
    double y = x[5];
    Eigen::Matrix3d G;
    G << cos(p)   , 0 , -cos(r)* sin(p),
            0     , 1 , sin(r),
            sin(p), 0 , cos(r) * cos(p);

    Eigen::Vector3d wm, x4, ng, x2d;

    wm = u.head(3);
    x4 = x.segment(9, 3);
    ng = n.head(3);

    x2d = G.inverse() * (wm - x4 - ng);

    xdot[3] = x2d[0];
    xdot[4] = x2d[1];
    xdot[5] = x2d[2];

    /* ---------- 3 ---------- */
    Eigen::Vector3d gravity, am, x5, na, x3d;
    Eigen::Matrix3d R;

    R<<     cos(y)*cos(p)-sin(r)*sin(y)*sin(p), -cos(r)*sin(y),cos(y)*sin(p)+cos(p)*sin(r)*sin(y),
            cos(p)*sin(y)+cos(y)*sin(r)*sin(p), cos(r)*cos(y) ,sin(y)*sin(p)-cos(y)*cos(p)*sin(r),
            -cos(r)*sin(p),                     sin(r)        ,cos(r)*cos(p);

    am = u.tail(3);
    x5 = x.segment(12, 3);
    na = n.segment(3, 3);
    gravity = Eigen::Vector3d(0, 0, -9.8);

    x3d = gravity + R * (am - x5 - na);

    xdot[6] = x3d[0];
    xdot[7] = x3d[1];
    xdot[8] = x3d[2];

    // n0:2 ngx, ngy, ngz
    // n3:5 nax, nay, naz
    // n6:8 nbgx, nbgy, nbgz
    // n9:11 nbax, nbay, nbaz
    /* ---------- 4 ---------- */
    xdot[9] = n[6];
    xdot[10] = n[7];
    xdot[11] = n[8];

    /* ---------- 5 ---------- */
    xdot[12] = n[9];
    xdot[13] = n[10];
    xdot[14] = n[11];

    return xdot;
}

Mat15x15 jacobian_A(const Vec15& x, const Vec6& u, const Vec12& n)
{
    Mat15x15 A;

    // x3:5   r,p,y
    // x9:11  bgx,bgy,bgz
    // x12:14 bax,bay,baz
    double r = x[3];
    double p = x[4];
    double y = x[5];
    double bgx = x[9];
    double bgy = x[10];
    double bgz = x[11];
    double bax = x[12];
    double bay = x[13];
    double baz = x[14];

    // u0:2 wmx, wmy, wmz
    // u3:5 amx, amy, amz
    double wmx, wmy, wmz, amx, amy, amz;
    wmx = u[0];
    wmy = u[1];
    wmz = u[2];
    amx = u[3];
    amy = u[4];
    amz = u[5];

    // n0:2 ngx, ngy, ngz
    // n3:5 nax, nay, naz
    // n6:8 nbgx, nbgy, nbgz
    // n9:11 nbax, nbay, nbaz
    double ngx, ngy, ngz, nax, nay, naz;
    ngx = n[0];
    ngy = n[1];
    ngz = n[2];
    nax = n[3];
    nay = n[4];
    naz = n[5];

    //Matlab simplify(jacobian([x_dot],[px py pz r p y vx vy vz bgx bgy bgz bax bay baz]))
       A<<     0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,
               0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,
               0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,
               0,0,0,0,bgx*sin(p)-ngz*cos(p)-bgz*cos(p)+wmz*cos(p)+ngx*sin(p)-wmx*sin(p),0,0,0,0,-cos(p),0,-sin(p),0,0,0,
               0,0,0,(bgz*cos(p)+ngz*cos(p)-bgx*sin(p)-wmz*cos(p)-ngx*sin(p)+wmx*sin(p))/cos(r)*cos(r),-(sin(r)*(bgx*cos(p)+ngx*cos(p)+bgz*sin(p)-wmx*cos(p)+ngz*sin(p)-wmz*sin(p)))/cos(r),0,0,0,0,-(sin(p)*sin(r))/cos(r),-1,(cos(p)*sin(r))/cos(r),0,0,0,
               0,0,0,-(sin(r)*(bgz*cos(p)+ngz*cos(p)-bgx*sin(p)-wmz*cos(p)-ngx*sin(p)+wmx*sin(p)))/cos(r)*cos(r),(bgx*cos(p)+ngx*cos(p)+bgz*sin(p)-wmx*cos(p)+ngz*sin(p)-wmz*sin(p))/cos(r),0,0,0,0,sin(p)/cos(r),0,-cos(p)/cos(r),0,0,0,
               0,0,0,cos(r)*sin(p)*sin(y)*(bax-amx+nax)-cos(p)*cos(r)*sin(y)*(baz-amz+naz)-sin(r)*sin(y)*(bay-amy+nay),(cos(y)*sin(p)+cos(p)*sin(r)*sin(y))*(bax-amx+nax)-(cos(p)*cos(y)-sin(p)*sin(r)*sin(y))*(baz-amz+naz),(cos(p)*sin(y)+cos(y)*sin(p)*sin(r))*(bax-amx+nax)+(sin(p)*sin(y)-cos(p)*cos(y)*sin(r))*(baz-amz+naz)+cos(r)*cos(y)*(bay-amy+nay),0,0,0,0,0,0,sin(p)*sin(r)*sin(y)-cos(p)*cos(y),cos(r)*sin(y),-cos(y)*sin(p)-cos(p)*sin(r)*sin(y),
               0,0,0,cos(y)*sin(r)*(bay-amy+nay)+cos(p)*cos(r)*cos(y)*(baz-amz+naz)-cos(r)*cos(y)*sin(p)*(bax-amx+nax),(sin(p)*sin(y)-cos(p)*cos(y)*sin(r))*(bax-amx+nax)-(cos(p)*sin(y)+cos(y)*sin(p)*sin(r))*(baz-amz+naz),cos(r)*sin(y)*(bay-amy+nay)-(cos(y)*sin(p)+cos(p)*sin(r)*sin(y))*(baz-amz+naz)-(cos(p)*cos(y)-sin(p)*sin(r)*sin(y))*(bax-amx+nax),0,0,0,0,0,0,-cos(p)*sin(y)-cos(y)*sin(p)*sin(r),-cos(r)*cos(y),cos(p)*cos(y)*sin(r)-sin(p)*sin(y),
               0,0,0,cos(p)*sin(r)*(baz-amz+naz)-cos(r)*(bay-amy+nay)-sin(p)*sin(r)*(bax-amx+nax),cos(r)*sin(p)*(baz-amz+naz)+cos(p)*cos(r)*(bax-amx+nax),0,0,0,0,0,0,0,cos(r)*sin(p),-sin(r),-cos(p)*cos(r),
               0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
               0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
               0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
               0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
               0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
               0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;

    return A;
}

Mat15x12 jacobian_U(const Vec15& x, const Vec6& u, const Vec12& n)
{
    Mat15x12 U;

    // x3:5 r,p,y
    double r = x[3];
    double p = x[4];
    double y = x[5];

    //Matlab simplify(jacobian([x_dot],[px py pz r p y vx vy vz bgx bgy bgz bax bay baz]))
    U<<     0,0,0,0,0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,0,0,0,0,
            0,0,0,0,0,0,0,0,0,0,0,0,
            -cos(p),0,-sin(p),0,0,0,0,0,0,0,0,0,
            -(sin(p)*sin(r))/cos(r),-1,(cos(p)*sin(r))/cos(r),0,0,0,0,0,0,0,0,0,
            sin(p)/cos(r),0,-cos(p)/cos(r),0,0,0,0,0,0,0,0,0,
            0,0,0,sin(p)*sin(r)*sin(y)-cos(p)*cos(y),cos(r)*sin(y),-cos(y)*sin(p)-cos(p)*sin(r)*sin(y),0,0,0,0,0,0,
            0,0,0,-cos(p)*sin(y)-cos(y)*sin(p)*sin(r),-cos(r)*cos(y),cos(p)*cos(y)*sin(r)-sin(p)*sin(y),0,0,0,0,0,0,
            0,0,0,cos(r)*sin(p),-sin(r),-cos(p)*cos(r),0,0,0,0,0,0,
            0,0,0,0,0,0,1,0,0,0,0,0,
            0,0,0,0,0,0,0,1,0,0,0,0,
            0,0,0,0,0,0,0,0,1,0,0,0,
            0,0,0,0,0,0,0,0,0,1,0,0,
            0,0,0,0,0,0,0,0,0,0,1,0,
            0,0,0,0,0,0,0,0,0,0,0,1;

    return U;
}

Mat6x15 jacobian_C_vo(const Vec15& x)
{
    Mat6x15 C;
    C<<     1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
            0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,
            0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,
            0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,
            0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,
            0,0,0,0,0,1,0,0,0,0,0,0,0,0,0;
    return C;
}

Mat6x6 jacobian_W_vo(const Vec15& x, const Vec6& v)
{
    Mat6x6 W;
    W<<     1,0,0,0,0,0,
            0,1,0,0,0,0,
            0,0,1,0,0,0,
            0,0,0,1,0,0,
            0,0,0,0,1,0,
            0,0,0,0,0,1;
    return W;
}

Vec3 velocity_obsrvation_in_camera(const Vec15& x, const Vec3& v)
{
  Eigen::Matrix3d Rx, Ry, Rz, Rx2;
  Eigen::Vector3d x3;

  Rx << 1, 0, 0, 0, cos(x[3]), -sin(x[3]), 0, sin(x[3]), cos(x[3]);
  Ry << cos(x[4]), 0.0, sin(x[4]), 0, 1, 0, -sin(x[4]), 0, cos(x[4]);
  Rz << cos(x[5]), -sin(x[5]), 0, sin(x[5]), cos(x[5]), 0, 0, 0, 1;
  Rx2 = Rz * Rx * Ry;

  x3 = x.segment(6, 3);

  Vec3 zt;
  zt.head(2) = (Rx2.transpose() * x3).head(2);
  zt(2) = x[2];

  zt += v;

  return zt;
}
