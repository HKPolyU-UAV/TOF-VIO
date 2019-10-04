#ifndef ESKF_IMU_H
#define ESKF_IMU_H

#include <include/common.h>
#include <sensor_msgs/Imu.h>
#include <deque>


using namespace std;
using namespace Eigen;

struct SYS_STATE
{
  double stamp;//time stamp
  Vec6  u;//imu input
  //u
  //[0 - 2]   wm  //gyro measurement
  //[3 - 5]   am  //acc  measurement
  Vec16 n_state;
  //nominal state
  //[0 - 3]   qw,qx,qy,qz
  //[4 - 6]   px,py,pz
  //[7 - 9]   vx,vy,vz
  //[10 -12]  bwx,bwy,bwz
  //[13-15]   bax,bay,baz
  Vec15 e_state;
  //error state
  //[0 - 2]   r,p,y
  //[3 - 5]   px,py,pz
  //[6 - 8]   vx,vy,vz
  //[9 -11]  bwx,bwy,bwz
  //[12-14]   bax,bay,baz
  Mat15x15 cov;//covariance matrix
};

class ESKF_IMU
{
public:

  double sigma_a,sigma_w,sigma_ab,sigma_wb;
  double sigma_q,sigma_p;

  deque<SYS_STATE> states;
  SYS_STATE curr_state;
  Mat15x15 cov_mat;
  Vec7 z;//error observation
  size_t innovation_index;
  double vo_delay;
  // the vo_delay is the time lag of the vo stamp than the acture capture time

  double imu_start_time;


  Mat15x15 Q_imu;
  Mat15x15 Q_icp;

  Mat3x3 I3x3;
  Mat4x4 I4x4;
  Mat15x15 I15x15;

  ESKF_IMU();
  ESKF_IMU(double sigma_a_in,
           double sigma_w_in,
           double sigma_ab_in,
           double sigma_wb_in,
           double sigma_icp_q_in,
           double sigma_icp_p_in,
           double vo_delay_in);

  void init_from_vo(double secs, double qw, double qx, double qy, double qz, double px, double py, double pz);

  //update
  void read_imu_msg(double secs, double ax, double ay, double az, double gx, double gy, double gz);
  void update_Nominal_Error_Cov();
  void update_Error_State_COV(const SYS_STATE& prev, SYS_STATE& curr);
  void update_Nominal_State(const SYS_STATE& prev, SYS_STATE& curr);


  //innovation
  void read_vo_msg(double secs, double qw, double qx, double qy, double qz, double px, double py, double pz);
  void innovate_ErrorState();
  void innovate_Inject_Reset();
  void innovate_reintegrate();


};

#endif // ESKF_IMU_H
