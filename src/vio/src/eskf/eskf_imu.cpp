#include <include/eskf_imu.h>

IOFormat CleanFmt(3, 0, "|", "\n", "[", "]");

inline Mat3x3 skew_symmetric_matrix(const Vec3 a)
{
  Mat3x3 A;
  A<< 0,     -a(2),   a(1),
      a(2),      0,  -a(0),
      -a(1),  a(0),     0;
  return A;
}

ESKF_IMU::ESKF_IMU()
{

}

ESKF_IMU::ESKF_IMU(double sigma_a_in,
                   double sigma_w_in,
                   double sigma_ab_in,
                   double sigma_wb_in,
                   double sigma_icp_q_in,
                   double sigma_icp_p_in,
                   double vo_delay_in)
{
  this->sigma_a = sigma_a_in;
  this->sigma_w = sigma_w_in;
  this->sigma_ab = sigma_ab_in;
  this->sigma_wb = sigma_wb_in;
  this->sigma_q = sigma_icp_q_in;
  this->sigma_p = sigma_icp_p_in;
  this->vo_delay = vo_delay_in*0.001;

  I3x3.setIdentity();
  I4x4.setIdentity();
  I15x15.setIdentity();

}

void ESKF_IMU::init_from_vo(double secs, double qw, double qx, double qy, double qz, double px, double py, double pz)
{
  cout << this->vo_delay << endl;
  cout << "init from vo at time" << (secs-imu_start_time) << endl;
  //find nearest state in states q
  size_t idx= this->states.size()-1;
  for (;idx>0; idx--)
  {
    if((secs-this->vo_delay)-this->states.at(idx).stamp>0)
      break;
  }
  cout << "states size is " << states.size() << ", the neareat state idx is " << idx << "  with the stamp " << states.at(idx).stamp-imu_start_time << endl;
  cout << qw << "  " << qx << "  " << qy << "  " << qz << "  "<< px << "  "<< py << "  "<< pz << endl;
  //set the nominal state and reset error state and it's cov
  Vec4 q=Vec4(qw,qx,qy,qz);
  Vec3 p=Vec3(px,py,pz);
  this->states.at(idx).n_state.head(4)=q;
  this->states.at(idx).n_state.segment(4,3)=p;
  this->states.at(idx).e_state.setZero();
  this->states.at(idx).cov.setIdentity();
  for(size_t i=idx; i<(this->states.size()-1); i++)
  {
    SYS_STATE old_state=this->states.at(i);
    SYS_STATE new_state=this->states.at(i+1);
    this->update_Nominal_State(old_state,new_state);
    this->update_Error_State_COV(old_state,new_state);
    this->states.at(i+1)=new_state;
  }

  //reintegration to uptodate state
}

/***************************************************************************************************************************/
//Update
/***************************************************************************************************************************/
void ESKF_IMU::read_imu_msg(double secs, double ax, double ay, double az, double gx, double gy, double gz)
{
  static bool first_imu_received=0;
  if(!first_imu_received)
  {
    this->imu_start_time = secs;
    first_imu_received=true;
  }
  //cout << secs-imu_start_time << endl;
  this->curr_state.u[0] = gx;
  this->curr_state.u[1] = gy;
  this->curr_state.u[2] = gz;
  this->curr_state.u[3] = ax;
  this->curr_state.u[4] = ay;
  this->curr_state.u[5] = az;
  this->curr_state.stamp = secs;
  //cout << this->curr_state.u.transpose().format(CleanFmt) << endl;
}

void ESKF_IMU::update_Nominal_Error_Cov()
{
  SYS_STATE prev = this->states.back();
  this->update_Nominal_State(prev,this->curr_state);
  this->update_Error_State_COV(prev,this->curr_state);
  this->states.push_back(this->curr_state);
  if(this->states.size()>=200)
  {
    states.pop_front();
  }
}

void ESKF_IMU::update_Nominal_State(const SYS_STATE& prev, SYS_STATE& curr)
{
  double dT=curr.stamp-prev.stamp;
  Vec4  q=prev.n_state.head(4);
  Vec3  p=prev.n_state.segment(4,3);
  Vec3  v=prev.n_state.segment(7,3);
  Vec3  wb=prev.n_state.segment(10,3);
  Vec3  ab=prev.n_state.segment(13,3);

  Vec3 wm=curr.u.head(3);
  Vec3 am=curr.u.tail(3);

  //q
  Vec3 w=wm-wb;
  Mat4x4 iq;
  //q_integration_matrix q*w = mat[w]*q where [] follow:
  //  0, -wx, -wy, -wz,
  // wx,   0,  wz, -wy,
  // wy, -wz,   0,  wx,
  // wz,  wy, -wx,   0,
  iq<<  0, -w(0), -w(1), -w(2),
      w(0),    0,  w(2), -w(1),
      w(1), -w(2),    0,  w(0),
      w(2),  w(1), -w(0),    0;
  Vec4 q_diff = iq*q*dT;
  Vec4 q_new_vec = q+q_diff;
  Quaterniond q_new(q_new_vec[0],q_new_vec[1],q_new_vec[2],q_new_vec[3]);
  q_new.normalize();
  //p
  Mat3x3 R = q_new.toRotationMatrix();
  Vec3 p_diff = dT*v;
  //Vec3 p_diff = dT*v + 0.5*dT*dT*(R*(am-ab)+Vec3(0,0,-9.8));
  Vec3 p_new = p+p_diff;
  //v
  Vec3 v_diff = dT*(R*(am-ab)+Vec3(0,0,-9.8));
  Vec3 v_new= v + v_diff;

  //wb and ab remain same
  curr.n_state.head(4)=Vec4(q_new.w(),q_new.x(),q_new.y(),q_new.z());
  curr.n_state.segment(4,3)=p_new;
  curr.n_state.segment(7,3)=v_new;
  curr.n_state.segment(10,3)=wb;
  curr.n_state.segment(13,3)=ab;
}

void ESKF_IMU::update_Error_State_COV(const SYS_STATE& prev, SYS_STATE& curr)
{
  double dT=curr.stamp-prev.stamp;
  //   0                3  6    9     12
  //0  I              | 0 |0   |−R∆t |0
  //3  0              | I |I∆t |0    |0
  //6  −[R(am−ab)]×∆t | 0 |I   |0    |−R∆t
  //9  0              | 0 |0   |I    |0
  //12 0              | 0 |0   |0    |I
  Vec4  n_q=this->curr_state.n_state.head(4);
  Eigen::Quaterniond q(n_q(0),n_q(1),n_q(2),n_q(3));
  Mat3x3 R=q.toRotationMatrix();
  Vec3 wm=this->curr_state.u.head(3);
  Vec3 am=this->curr_state.u.tail(3);
  Vec3 wb=this->curr_state.n_state.segment(10,3);
  Vec3 ab=this->curr_state.n_state.segment(13,3);

  Mat15x15 F;
  F.setZero();
  F.block<3,3>(0,0)=I3x3;
  F.block<3,3>(0,9)=-R*dT;
  F.block<3,3>(3,3)=I3x3;
  F.block<3,3>(3,6)=I3x3*dT;
  F.block<3,3>(6,0)=-skew_symmetric_matrix(R*(am-ab))*dT;
  F.block<3,3>(6,6)=I3x3;
  F.block<3,3>(6,12)=-R*dT;
  F.block<3,3>(9,9)=I3x3;
  F.block<3,3>(12,12)=I3x3;
  //  cout << "F" << endl << F << endl;
  Mat15x12 Fi;
  Fi.setZero();
  Fi.block<3,3>(0,0) =I3x3;
  Fi.block<3,3>(6,3) =I3x3;
  Fi.block<3,3>(9,6) =I3x3;
  Fi.block<3,3>(12,9)=I3x3;
  //  cout << "Fi" << endl << Fi << endl;

  Mat12x12 Qimu;
  Qimu.setZero();
  Qimu.block<3,3>(0,0) = this->sigma_w*(dT*dT)*I3x3;
  Qimu.block<3,3>(3,3) = this->sigma_a*(dT*dT)*I3x3;
  Qimu.block<3,3>(6,6) = this->sigma_wb*dT*I3x3;
  Qimu.block<3,3>(9,9) = this->sigma_ab*dT*I3x3;
  //  cout << "Qimu" << endl << Qimu << endl;

  curr.e_state = prev.e_state + F* prev.e_state;
  curr.cov = (F*prev.cov*(F.transpose()))+(Fi*Qimu*Fi.transpose());
  //cout << (Fi*Qimu*Fi.transpose()) << endl;
}


/***************************************************************************************************************************/
//Innovate
/***************************************************************************************************************************/

void ESKF_IMU::read_vo_msg(double secs, double qw, double qx, double qy, double qz, double px, double py, double pz)
{
  //find innovation index
  this->innovation_index = this->states.size()-1;
  size_t idx= this->states.size()-1;
  for (;idx>0; idx--)
  {
    if(secs-this->vo_delay-this->states.at(idx).stamp>0)
      break;
  }
  //cout << qw << "  " << qx << "  " << qy << "  " << qz << "  "<< px << "  "<< py << "  "<< pz << endl;
  //cout << "states size is " << states.size() << ", the neareat state idx is " << idx << "  with the stamp " << states.at(idx).stamp-imu_start_time << endl;
  innovation_index = idx;
  //cout << this->states.at(innovation_index).n_state.transpose() << endl;
  this->z[0]=qw-this->states.at(innovation_index).n_state[0];
  this->z[1]=qx-this->states.at(innovation_index).n_state[1];
  this->z[2]=qy-this->states.at(innovation_index).n_state[2];
  this->z[3]=qz-this->states.at(innovation_index).n_state[3];
  this->z[4]=px-this->states.at(innovation_index).n_state[4];
  this->z[5]=py-this->states.at(innovation_index).n_state[5];
  this->z[6]=pz-this->states.at(innovation_index).n_state[6];
  cout << "z: " << this->z.transpose() << endl;
}

void ESKF_IMU::innovate_ErrorState()
{
  SYS_STATE nearest_state = this->states.at(this->innovation_index);
  Vec4 q=nearest_state.n_state.head(4);
  Mat4x3 n_over_e_matrix;
  n_over_e_matrix<< -q[1], -q[2], -q[3],  q[0], -q[3],  q[2],   q[3], q[0],  -q[1],   -q[2], q[1],  q[0];

  Mat7x15 H;
  H.setZero();
  H.block<4,3>(0,0) = 0.5*n_over_e_matrix;
  H.block<3,3>(4,3) =I3x3;
  //cout << "H:" << endl <<H.format(CleanFmt) << endl;
  //cout << "COV:" << endl << nearest_state.cov.format(CleanFmt) << endl;
  Mat7x7 Qvo;
  Qvo.setIdentity();
  Qvo.block<4,4>(0,0)=I4x4*sigma_q;
  Qvo.block<3,3>(4,4)=I3x3*sigma_p;
  //cout << "Qvo:" << endl <<Qvo.format(CleanFmt) << endl;
  Mat15x7 K;
  Mat7x7 covZ;
  covZ = H*nearest_state.cov*H.transpose()+Qvo;
  K=nearest_state.cov*H.transpose()*(covZ.inverse());
  //cout << "K:" << endl <<K.format(CleanFmt) << endl;

  //cout << "eso:" << nearest_state.e_state.transpose().format(CleanFmt) << endl;

  //cout << "K:" << endl <<K << endl;
  nearest_state.e_state=K*this->z;
  nearest_state.cov=nearest_state.cov-K*covZ*K.transpose();
  //  cout << "COV:" << endl << nearest_state.cov << endl;
  this->states.at(this->innovation_index) = nearest_state;
  //cout << "esn:" << nearest_state.e_state.transpose().format(CleanFmt) << endl;

}

void ESKF_IMU::innovate_Inject_Reset()
{

  SYS_STATE nearest_state = this->states.at(this->innovation_index);
  cout << "injection of error state at idx: " << this->innovation_index << endl;
  Vec4 q_vec;
  Vec3 etheta;
  Quaterniond q,ethetaq,nq;
  Vec12 t_s12,e_s12,n_s12;

  q_vec=nearest_state.n_state.head(4);
  q = Quaterniond(q_vec[0],q_vec[1],q_vec[2],q_vec[3]);
  q.normalize();
  etheta =nearest_state.e_state.head(3);
  ethetaq = Quaterniond(1,etheta[0]/2,etheta[1]/2,etheta[2]/2);
  ethetaq.normalize();
  nq=ethetaq*q;

  n_s12 = nearest_state.n_state.tail(12);
  e_s12 = nearest_state.e_state.tail(12);
  t_s12 = n_s12+e_s12;

  //cout << "nso:" << nearest_state.n_state.transpose().format(CleanFmt) << endl;
  nearest_state.n_state.head(4)=Vec4(nq.w(),nq.x(),nq.y(),nq.z());
  nearest_state.n_state.tail(12)=t_s12;
  //cout << "nsn:" <<  nearest_state.n_state.transpose().format(CleanFmt) << endl;

  Mat3x3 G;
  G=I3x3+skew_symmetric_matrix(0.5*etheta);
  Mat3x3 cov_theta_block = nearest_state.cov.block<3,3>(0,0);
  nearest_state.cov.block<3,3>(0,0)= G*cov_theta_block*(G.transpose());
  nearest_state.e_state.setZero();

  this->states.at(this->innovation_index)=nearest_state;
}

void ESKF_IMU::innovate_reintegrate()
{//From Innovation Point to Uptodate state;
  for(size_t i=this->innovation_index; i<(this->states.size()-1); i++)
  {
    SYS_STATE old_state=this->states.at(i);
    SYS_STATE new_state=this->states.at(i+1);
    this->update_Nominal_State(old_state,new_state);
    this->update_Error_State_COV(old_state,new_state);
    this->states.at(i+1)=new_state;
  }
}





